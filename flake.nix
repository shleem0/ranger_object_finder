{
  description = "Ranger development flake";

  inputs.haskellNix.url =
    "github:input-output-hk/haskell.nix?rev=29be2a6f5fec17f7385128eee8d8fac28ca65bfa";
  inputs.nixpkgs.follows = "haskellNix/nixpkgs-2411";
  inputs.nix-ros-overlay.url = "github:lopsided98/nix-ros-overlay/master";
  inputs.nixpkgs-ros.follows = "nix-ros-overlay/nixpkgs";

  outputs = { nixpkgs-ros, nixpkgs, haskellNix, nix-ros-overlay, ... }:
    nix-ros-overlay.inputs.flake-utils.lib.eachDefaultSystem (system:
      let
        pkgs = import nixpkgs {
          inherit system overlays;
          inherit (haskellNix) config;
        };

        overlays = [
          haskellNix.overlay
          (final: prev: {
            ranger-daemon = final.haskell-nix.cabalProject' {
              src = pkgs.lib.fileset.toSource {
                root = ./.;
                fileset = ./ranger-daemon;
              };
              cabalProject = builtins.readFile ./cabal.project;
              cabalProjectFreeze = builtins.readFile ./cabal.project.freeze;
              compiler-nix-name = "ghc8107";
              shell.tools = {
                cabal = { };
                haskell-language-server = {
                  src = final.haskell-nix.sources."hls-2.2";
                };
              };
            };
          })
        ];
        pkgs-ros = import nixpkgs-ros {
          inherit system;
          overlays = [ nix-ros-overlay.overlays.default ];
        };

        ros-packages = with pkgs-ros; [
          colcon
          (with rosPackages.humble; buildEnv { paths = [ ros-core ]; })
        ];

        flake = pkgs.ranger-daemon.flake { };

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
