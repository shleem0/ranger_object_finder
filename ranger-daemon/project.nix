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
  cabalProjectFreeze = builtins.readFile ../cabal.project.freeze;
  cabalProject = builtins.readFile ../cabal.project;
  shell.buildInputs = [
    (pkgs.writeScriptBin "haskell-language-server-wrapper" ''
      #!${pkgs.stdenv.shell}
      exec haskell-language-server "$@"
    '')
  ];
}
