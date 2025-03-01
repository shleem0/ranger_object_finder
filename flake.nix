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
      builderSystem = "x86_64-linux";
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

      nixosConfigurations.sdp = lib.nixosSystem {
        system = raspiSystem;
        specialArgs = {
          inherit base-home;
          programsdb = programsdb raspiSystem;
        };
        modules = [
          "${nixpkgs}/nixos/modules/installer/sd-card/sd-image-aarch64.nix"
          nixosModules.sdp
          raspi-3
          ({ lib, ... }: {
            # Disable zfs (kernel must be built, takes ages)
            boot.supportedFilesystems.zfs = lib.mkForce false;
            environment.systemPackages = [
              self.outputs.packages.${builderSystem}."aarch64-unknown-linux-gnu:ranger-daemon:exe:ranger-daemon"
              self.outputs.packages.${raspiSystem}.ranger-object-recognition
            ];
          })
        ];
      };

      # nix run .#nixosConfigurations.sdp-local.config.system.build.nixos-shell
      nixosConfigurations.sdp-local = lib.nixosSystem {
        system = builderSystem;
        specialArgs = {
          inherit base-home;
          programsdb = programsdb builderSystem;
        };
        modules = [
          nixosModules.sdp
          hm
          nixos-shell.nixosModules.nixos-shell
          {
            # https://github.com/Mic92/nixos-shell/pull/89
            # lol
            networking.hostName = lib.mkForce "nixos";
            environment.systemPackages = [
              self.outputs.packages.${builderSystem}."ranger-daemon:exe:ranger-daemon"
              self.outputs.packages.${builderSystem}.ranger-object-recognition
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

        ranger-daemon-flake = (import ranger-daemon/project.nix pkgs).flake {
          crossPlatforms = p: [ p.aarch64-multiplatform ];
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
