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
          overlays = [ haskellNix.overlay self.outputs.overlay ];
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

    in rec {
      nixosModules.sdp = { imports = [ ./ranger-nixos/sdp.nix hm ]; };

      nixosModules.ranger = { pkgs, ... }: {
        environment.systemPackages = [
          # TODO: replace with default package
          self.outputs.packages.${pkgs.system}."ranger-daemon:exe:ranger-daemon"
          self.outputs.packages.${pkgs.system}.ranger-object-recognition
        ];
      };

      nixosConfigurations.sdp = lib.nixosSystem rec {
        system = raspiSystem;
        specialArgs = {
          inherit base-home;
          programsdb = programsdb system;
        };
        modules = [
          "${nixpkgs}/nixos/modules/installer/sd-card/sd-image-aarch64.nix"
          nixosModules.sdp
          nixosModules.ranger
          raspi-3
        ];
      };

      # The aarch64 version of ranger-daemon will take ages (at least 12 hours) to compile
      # because you will have to compile ghc. This configuration excludes it
      nixosConfigurations.sdp-no-ranger = lib.nixosSystem rec {
        system = raspiSystem;
        specialArgs = {
          inherit base-home;
          programsdb = programsdb system;
        };
        modules = [
          "${nixpkgs}/nixos/modules/installer/sd-card/sd-image-aarch64.nix"
          nixosModules.sdp
          raspi-3
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
          nixosModules.sdp
          nixosModules.ranger
          hm
          nixos-shell.nixosModules.nixos-shell
          {
            # https://github.com/Mic92/nixos-shell/pull/89
            # lol
            networking.hostName = lib.mkForce "nixos";
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

      # Auxiliary packages
      overlay = final: prev: {
        ranger-object-recognition =
          final.callPackage ./ranger_object_recognition/package.nix { };
      };
    }
    # Shells + daemon
    // nix-ros-overlay.inputs.flake-utils.lib.eachDefaultSystem (system:
      let
        pkgs = pkgs-hs system;

        ranger-daemon-flake = (import ranger-daemon/project.nix pkgs).flake { };

        pkgs-ros = import nixpkgs-ros {
          inherit system;
          overlays = [ nix-ros-overlay.overlays.default ];
        };

        ros-packages = with pkgs-ros; [
          colcon
          (with rosPackages.humble;
            buildEnv { paths = [ ros-core slam-toolbox ]; })
        ];

      in rec {
        devShells.default = pkgs.mkShell {
          name = "ROS + daemon combined shell";
          inputsFrom = [ devShells.daemon devShells.ros ];
        };

        devShells.daemon = ranger-daemon-flake.devShells.default;

        devShells.ros = pkgs-ros.mkShell {
          name = "ROS development";
          buildInputs = ros-packages;
        };

        packages = ranger-daemon-flake.packages // {
          inherit (pkgs) ranger-object-recognition;
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
