{ stdenv, ranger-lib, pkgs, ... }:

stdenv.mkDerivation {
  src = ./.;
  name = "ranger-daemon-ffi-test";
  buildPhase = ''
    ghc -no-hs-main main.c ${ranger-lib}/lib/${pkgs.system}-ghc-8.10.7/ranger-daemon*/*.o -I${ranger-lib}/include -o ranger-daemon-ffi-test
  '';
  installPhase = ''
    mkdir -p $out/bin
    cp ranger-daemon-ffi-test $out/bin
  '';
  nativeBuildInputs = ranger-lib.nativeBuildInputs;
  allowSubstitutes = false;
}
