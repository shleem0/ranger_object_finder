pkgs:

let
  asSubdirectory = pkgs.lib.fileset.toSource {
    fileset = ./../ranger-daemon;
    root = ./..;
  };
in pkgs.haskell-nix.cabalProject' {
  src = pkgs.haskell-nix.cleanSourceHaskell {
    src = asSubdirectory;
    name = "ranger-daemon-src";
  };
  compiler-nix-name = "ghc8107";
  shell.tools = {
    cabal = { };
    haskell-language-server = { src = pkgs.haskell-nix.sources."hls-2.2"; };
  };
  modules = [{
    packages.ranger-daemon.postInstall = ''
      ${pkgs.tree}/bin/tree
      mkdir -p $out/include/Ranger
      cp dist/build/Ranger/CApi_stub.h $out/include/Ranger || true
      cp cbits/ranger_types.h $out/include || true
    '';
  }];
  cabalProject = builtins.readFile ../cabal.project;
  shell.buildInputs = [
    (pkgs.writeScriptBin "haskell-language-server-wrapper" ''
      #!${pkgs.stdenv.shell}
      exec haskell-language-server "$@"
    '')
  ];
}
