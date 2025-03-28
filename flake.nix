{
  description = "Ranger development flake";

  inputs = {
    haskellNix.url =
      "github:input-output-hk/haskell.nix?rev=29be2a6f5fec17f7385128eee8d8fac28ca65bfa";

    nixpkgs.follows = "haskellNix/nixpkgs-2411";

    nix-ros-overlay.url = "github:lopsided98/nix-ros-overlay/develop";

    home-manager = {
      url = "home-manager/release-24.11";
      inputs.nixpkgs.follows = "nixpkgs";
    };

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

    sops-nix.url = "github:Mic92/sops-nix";
    sops-nix.inputs.nixpkgs.follows = "nixpkgs";
  };

  outputs = { self, nixpkgs, haskellNix, nix-ros-overlay, fps, nixos-user
    , home-manager, deploy-rs, nixos-shell, sops-nix, ... }:
    let
      raspiSystem = "aarch64-linux";
      lib = nixpkgs.lib;
      mkProgramsdb = system: fps.packages.${system}.programs-sqlite;
      hm = home-manager.nixosModules.home-manager;
      base-home = nixos-user.nixosModules.cli;
      sops = sops-nix.nixosModules.sops;
      mkPkgs = system:
        import nixpkgs {
          system = system;
          overlays = [
            haskellNix.overlay
            nix-ros-overlay.overlays.default
            self.outputs.overlays.${system}.default
          ];
          inherit (haskellNix) config;
        };
      # Use nixpkgs binary cache for deploy-rs
      deployPkgs = import nixpkgs {
        system = raspiSystem;
        overlays = [
          deploy-rs.overlay
          (self: super: {
            deploy-rs = {
              inherit (mkPkgs raspiSystem) deploy-rs;
              lib = super.deploy-rs.lib;
            };
          })
        ];
      };

    in {
      deploy.nodes.sdp = {
        hostname = "sdp-ranger";
        profiles.system = {
          sshUser = "pi";
          user = "root";
          path = deployPkgs.deploy-rs.lib.activate.nixos
            self.nixosConfigurations.x86_64-linux.sdp;
        };
      };
    } // nix-ros-overlay.inputs.flake-utils.lib.eachDefaultSystem (system:
      let
        pkgs = mkPkgs system;

        ranger-daemon-flake = (import ranger-daemon/project.nix pkgs).flake {
          crossPlatforms = p: [ p.aarch64-multiplatform ];
        };

        ros-packages = pkgs:
          with pkgs; [
            colcon
            (with rosPackages.jazzy; buildEnv { paths = [ ros-core ]; })
          ];

      in rec {
        nixosModules.sdp = { imports = [ ./ranger-nixos/sdp.nix hm ]; };

        nixosConfigurations.sdp = lib.nixosSystem {
          system = raspiSystem;
          specialArgs = {
            inherit base-home;
            programsdb = mkProgramsdb raspiSystem;
          };
          modules = [
            "${nixpkgs}/nixos/modules/installer/sd-card/sd-image-aarch64.nix"
            nixosModules.sdp
            sops
            ./ranger-nixos/wifi.nix
            ({ pkgs, lib, ... }: {
              nixpkgs.overlays = [ nix-ros-overlay.overlays.default ];
              # Disable zfs (kernel must be built, takes ages)
              boot.supportedFilesystems.zfs = lib.mkForce false;
              environment.systemPackages = [
                self.outputs.packages.${system}."aarch64-unknown-linux-gnu:ranger-daemon:exe:ranger-daemon"
                self.outputs.packages.${raspiSystem}.ranger-object-recognition
              ] ++ ros-packages pkgs;
            })
          ];
        };

        # nix run .#nixosConfigurations.sdp-local.config.system.build.nixos-shell
        nixosConfigurations.sdp-local = lib.nixosSystem {
          inherit system;
          specialArgs = {
            inherit base-home;
            programsdb = mkProgramsdb system;
          };
          modules = [
            nixosModules.sdp
            hm
            nixos-shell.nixosModules.nixos-shell
            ({ pkgs, lib, ... }: {
              nixpkgs.overlays = [ nix-ros-overlay.overlays.default ];
              # https://github.com/Mic92/nixos-shell/pull/89
              # lol
              networking.hostName = lib.mkForce "nixos";
              environment.systemPackages = [
                self.outputs.packages.${system}."ranger-daemon:exe:ranger-daemon"
                self.outputs.packages.${system}.ranger-object-recognition
              ] ++ ros-packages pkgs;
            })
          ];
        };

        devShells.default = pkgs.mkShell {
          name = "ROS + daemon combined shell";
          inputsFrom = [ devShells.daemon devShells.ros ];
        };

        devShells.daemon = ranger-daemon-flake.devShells.default;

        devShells.ros = pkgs.mkShell {
          name = "ROS development";
          buildInputs = ros-packages pkgs;
        };

        devShells.bare = pkgs.mkShell {
          name = "Shell for manual compilation";
          buildInputs = with pkgs; [
            zlib
            dbus
            haskell.compiler.ghc810
            cabal-install
          ];
        };

        overlays.default = final: prev: {
          ranger-object-recognition =
            final.callPackage ./ranger_object_recognition/package.nix { };
          ranger-daemon-ffi-test =
            final.callPackage ./ranger-daemon-ffi-test/package.nix {
              ranger-lib =
                ranger-daemon-flake.packages."ranger-daemon:lib:ranger-daemon";
            };
        };

        packages = ranger-daemon-flake.packages // {
          inherit (pkgs) ranger-object-recognition ranger-daemon-ffi-test;
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
