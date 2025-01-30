{
  description = "Ranger development flake";
  inputs.nix-ros-overlay.url = "github:lopsided98/nix-ros-overlay/master";
  inputs.nixpkgs.follows = "nix-ros-overlay/nixpkgs";

  outputs = { nixpkgs, nix-ros-overlay, ... }:
    nix-ros-overlay.inputs.flake-utils.lib.eachDefaultSystem (system:
      let
        pkgs = import nixpkgs {
          inherit system;
          overlays = [ nix-ros-overlay.overlays.default ];
        };
        simpleble = pkgs.callPackage ./pkgs/simpleble.nix { };
        compile-packages = with pkgs; [ simpleble zlib cabal-install ghc ];
        ros-packages = with pkgs; [
          colcon
          (with rosPackages.humble; buildEnv { paths = [ ros-core ]; })
        ];
      in {
        devShells.default = pkgs.mkShell {
          name = "Ranger compile shell";
          packages = compile-packages ++ ros-packages;
        };
      });
}
