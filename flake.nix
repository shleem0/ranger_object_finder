{
  description = "Ranger development flake";

  inputs = {
    haskellNix.url =
      "github:input-output-hk/haskell.nix?rev=29be2a6f5fec17f7385128eee8d8fac28ca65bfa";

    nixpkgs.follows = "haskellNix/nixpkgs-2411";

    nix-ros-overlay.url = "github:lopsided98/nix-ros-overlay/master";

    nixpkgs-ros.follows = "nix-ros-overlay/nixpkgs";

    home-manager = {
      url = "home-manager/release-24.11";
      inputs.nixpkgs.follows = "nixpkgs";
    };

    nixos-hardware.url = "github:NixOS/nixos-hardware";

    deploy-rs = {
      url = "github:serokell/deploy-rs";
      inputs.nixpkgs.follows = "nixpkgs";
    };

    nixos-user.url = "github:ryndubei/nixos-user";
    nixos-user.inputs.nixpkgs.follows = "nixpkgs";
    nixos-user.inputs.home-manager.follows = "home-manager";

    nixos-shell.url = "github:Mic92/nixos-shell";
    nixos-shell.inputs.nixpkgs.follows = "nixpkgs";

    fps.url = "github:wamserma/flake-programs-sqlite";
    fps.inputs.nixpkgs.follows = "nixpkgs";
  };

  outputs = { self, nixpkgs-ros, nixpkgs, haskellNix, nix-ros-overlay, fps
    , nixos-user, home-manager, nixos-hardware, deploy-rs, nixos-shell, ... }:
    let
      raspiSystem = "aarch64-linux";
      lib = nixpkgs.lib;
      programsdb = system: fps.packages.${system}.programs-sqlite;
      hm = home-manager.nixosModules.home-manager;
      raspi-3 = nixos-hardware.nixosModules.raspberry-pi-3;
      base-home = nixos-user.nixosModules.cli;
      pkgs-hs = system:
        import nixpkgs {
          system = system;
          overlays = [ haskellNix.overlay ];
          inherit (haskellNix) config;
        };
      # Use nixpkgs binary cache for deploy-rs
      deployPkgs = import nixpkgs {
        system = raspiSystem;
        overlays = [
          deploy-rs.overlay
          (self: super: {
            deploy-rs = {
              inherit (pkgs-hs raspiSystem) deploy-rs;
              lib = super.deploy-rs.lib;
            };
          })
        ];
      };

    in {
      nixosConfigurations.sdp = lib.nixosSystem rec {
        system = raspiSystem;
        specialArgs = {
          inherit base-home;
          programsdb = programsdb system;
        };
        modules = [
          "${nixpkgs}/nixos/modules/installer/sd-card/sd-image-aarch64.nix"
          ./ranger-nixos/sdp.nix
          raspi-3
          hm
          {
            # TODO: replace with default package
            environment.systemPackages = [
              self.outputs.packages.${system}."ranger-daemon:exe:ranger-daemon"
            ];
          }
        ];
      };

      # nix run .#nixosConfigurations.sdp-local.config.system.build.nixos-shell
      nixosConfigurations.sdp-local = lib.nixosSystem rec {
        system = "x86_64-linux";
        specialArgs = {
          inherit base-home;
          programsdb = programsdb system;
        };
        modules = [
          ./ranger-nixos/sdp.nix
          hm
          nixos-shell.nixosModules.nixos-shell
          {
            # https://github.com/Mic92/nixos-shell/pull/89
            # lol
            networking.hostName = lib.mkForce "nixos";
            environment.systemPackages = [
              self.outputs.packages.${system}."ranger-daemon:exe:ranger-daemon"
            ];
          }
        ];
      };

      deploy.nodes.sdp = {
        hostname = "sdp-ranger"; # TODO: find appropriate ssh alias definition
        profiles.system = {
          sshUser = "pi";
          user = "root";
          path = deployPkgs.deploy-rs.lib.activate.nixos
            self.nixosConfigurations.sdp;
        };
      };
    }
    # Packages and shells
    // nix-ros-overlay.inputs.flake-utils.lib.eachDefaultSystem (system:
      let
        pkgs = pkgs-hs system;
        ranger-daemon = pkgs.haskell-nix.cabalProject' {
          src = lib.fileset.toSource {
            root = ./.;
            fileset = ./ranger-daemon;
          };
          cabalProject = builtins.readFile ./cabal.project;
          cabalProjectFreeze = builtins.readFile ./cabal.project.freeze;
          compiler-nix-name = "ghc8107";
          shell.tools = {
            cabal = { };
            haskell-language-server = {
              src = pkgs.haskell-nix.sources."hls-2.2";
            };
          };
          shell.buildInputs = [
            (pkgs.writeScriptBin "haskell-language-server-wrapper" ''
              #!${pkgs.stdenv.shell}
              exec haskell-language-server "$@"
            '')
          ];
        };

        pkgs-ros = import nixpkgs-ros {
          inherit system;
          overlays = [ nix-ros-overlay.overlays.default ];
        };

        ros-packages = with pkgs-ros; [
          colcon
          (with rosPackages.humble;
            buildEnv { paths = [ ros-core slam-toolbox ]; })
        ];

        flake = ranger-daemon.flake { };

      in flake // rec {
        devShells.default = pkgs.mkShell {
          name = "ROS + daemon combined shell";
          inputsFrom = [ devShells.daemon devShells.ros ];
        };

        devShells.daemon = flake.devShells.default;

        devShells.ros = pkgs-ros.mkShell {
          name = "ROS development";
          buildInputs = ros-packages;
        };
      });

  nixConfig = {
    extra-substituters = [
      "https://cache.iog.io"
      "https://cache.zw3rk.com"
      "https://ros.cachix.org"
    ];
  };
}
