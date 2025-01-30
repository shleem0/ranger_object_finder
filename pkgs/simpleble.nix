{ stdenv, lib, cmake, fmt, pkg-config, dbus, ... }:

stdenv.mkDerivation {
  src = builtins.fetchGit {
    url = "https://github.com/SimpleBLE/SimpleBLE.git";
    ref = "0.8.1";
    rev = "5d19f5048408fb7a273bc55de5c3a816f4ff417d";
  };

  name = "simpleble";
  version = "0.8.1";

  nativeBuildInputs = [ cmake fmt pkg-config dbus ];

  cmakeFlags = [
    "-S /build/source/simpleble -DSIMPLEBLE_USE_SESSION_DBUS=TRUE -DLIBFMT_VENDORIZE=False"
  ];

  cmakeBuildDir = "build_simpleble";

  meta = with lib; {
    description = "Library and bindings for Bluetooth Low Energy (BLE)";
    homepage = "https://github.com/SimpleBLE/SimpleBLE";
    license = licenses.gpl3;
    platforms = platforms.linux;
  };

  allowSubstitutes = false;
}
