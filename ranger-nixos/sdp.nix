{ pkgs, base-home, programsdb, ... }:

let
  # TODO: more SSH keys
  ssh-keys = [
    # vasily
    "ssh-rsa AAAAB3NzaC1yc2EAAAADAQABAAABgQC5mH6j/tcTVrddxLJprJhh/XN4jwwpCYN2xsyMqI2nIWvmtXggbNK8izG4gpRBIUP3RPKGFflAEM8wKi+O1nTwDvTyp3ciJlMSbreujsnI5Uox3Ca6Dk74/9z+F7rcmXBCJlB0KBEB1v8mhQk6Lm8kRXP0lQStFuerdCyJYEEM8pQwBYtOVM/Dqp93pnLVGgD7EKLcmxWLF4g82Jx/JjSplNT19y0j14Z0Qp9TEpVe3mx51L86G0Yn30DAMDVQO5EzZUlRSEo4KxvNJCz/fpC+hfw7EZ92Yc0gF8HjfMBaKJaqtv+TpxLZMvNwE59vFnG4FyN6jLhmxOLq9Rgx1iCmiG/f7cmykDqcy5BVkzEPdVuWdRNCzhCari4Wrq4RJsRMYSCVDMEtu+Swwi0cSJe79tNBFQKC8QR9lhMEEAwNmMB+gDykH8m6J66DHLG5qb96IYbfDPTlc5PprjlTgFezzY6xuQrmyo1Dlw18AJ/H3HhJM5n2gijjcxKddTkXoo0= vasilysterekhov@nixos-laptop"
    "ssh-rsa AAAAB3NzaC1yc2EAAAADAQABAAABgQCj7VuKLTmExfro8brkb2EiZ4Ce+RWRILcZ24K4n+tfmOWuNFT9j07HNoTDXocMKYp4ycUxdAmHH1wtte8hPcovjPoUJQxDvNPGUWXgiFt7us6ngyNaOp4dRX5ViW4diwvbd5djhK2b5X05tWoLw2Z45mB81VxiM4yI0vSAIr4u/BDnf2SlcPJBM1sro94QDKjKL/zIiaHFPbgeBcWkL5Lm/tEmE5JsfkKBoUPJoEmDQxG5gtqi2p8d23SzPV53dqbTK6sNnpqrhmmxpuZ26gqqXa/LW3mjRE4Q/RD/BlSnVUQkAzp9fQvuNgIswclevUhGlgeDOA+dVF+XR98VPJWR2ylx0U7g3FUBnBIyM8/0BSu/XdVzFVLobtv2qZsWVTAH/KwD4K7Ktmr9HnxUeCbfCxAmz45PWawrAdlIHPmARuy/hhIi0c53FXrjWDW2fE99usMBSOT6ADnLGa7VlVSISDL3lL2ftQFu0TLAVkmqoFvNI1YRkYAi3ooFE5dNi3E= vasilysterekhov@nixos-desktop"
  ];

  adminUser = name: {
    users.users.${name} = {
      isNormalUser = true;
      extraGroups = [ "wheel" ];
      openssh.authorizedKeys.keys = ssh-keys;
      password = "group13";
      linger = true;
    };
    home-manager.users.${name} = { ... }: {
      imports = [ base-home ];
      home.username = name;
      home.homeDirectory = "/home/${name}";
      home.stateVersion = "24.05";
    };
  };

  adminUsers = names: builtins.foldl' (acc: n: acc // (adminUser n)) { } names;
in {
  networking.hostName = "sdp-ranger";

  time.timeZone = "Europe/London";

  # Internationalisation properties.
  i18n.defaultLocale = "en_GB.UTF-8";

  # Disable adding more users
  users.mutableUsers = false;

  security.sudo.wheelNeedsPassword = false;

  environment.systemPackages = with pkgs; [
    libraspberrypi
    distrobox
    podman-tui
  ];

  virtualisation.containers.enable = true;

  virtualisation.podman = {
    enable = true;
    dockerCompat = true;
  };

  services.openssh = {
    enable = true;
    settings = {
      PasswordAuthentication = false;
      KbdInteractiveAuthentication = false;
      PermitRootLogin = "no";
    };
  };

  programs.git.enable = true;

  # Enable Tailscale LAN-over-wireguard
  services.tailscale.enable = true;
  # Opt out of sending client logs to Tailscale
  services.tailscale.extraDaemonFlags = [ "--no-logs-no-support" ];

  # Make command-not-found work with flakes
  # https://blog.nobbz.dev/2023-02-27-nixos-flakes-command-not-found/
  environment.etc."programs.sqlite".source = programsdb;
  programs.command-not-found.dbPath = "/etc/programs.sqlite";

  # Automatically connect to hotspots with this name and password
  networking.wireless.enable = true;
  networking.wireless.networks."ranger-hotspot".psk = "12345678";

  system.stateVersion = "24.05";

} // (adminUsers [
  "sholto"
  "kian"
  "remi"
  "bruce"
  "eric"
  "pelayo"
  "pani"
  "vasily"
])
