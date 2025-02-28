{ pkgs, ... }:

with pkgs.python311Packages;

buildPythonApplication {
  name = "ranger_object_recognition";

  src = ./.;

  propagatedBuildInputs = [ numpy tensorflow opencv4 matplotlib ];

  meta = { description = "Object recognition for Ranger"; };
}
