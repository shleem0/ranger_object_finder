pkgs:

pkgs.haskell-nix.cabalProject' {
  src = pkgs.haskell-nix.cleanSourceHaskell {
    src = ./.;
    name = "ranger-daemon-src";
  };
  compiler-nix-name = "ghc8107";
  shell.tools = {
    cabal = { };
    haskell-language-server = { src = pkgs.haskell-nix.sources."hls-2.2"; };
  };
  shell.buildInputs = [
    (pkgs.writeScriptBin "haskell-language-server-wrapper" ''
      #!${pkgs.stdenv.shell}
      exec haskell-language-server "$@"
    '')
  ];
}
