{ pkgs, base-home, programsdb, ... }:

let
  # TODO: more SSH keys
  ssh-keys = [
    # vasily
    "ssh-rsa AAAAB3NzaC1yc2EAAAADAQABAAABgQC5mH6j/tcTVrddxLJprJhh/XN4jwwpCYN2xsyMqI2nIWvmtXggbNK8izG4gpRBIUP3RPKGFflAEM8wKi+O1nTwDvTyp3ciJlMSbreujsnI5Uox3Ca6Dk74/9z+F7rcmXBCJlB0KBEB1v8mhQk6Lm8kRXP0lQStFuerdCyJYEEM8pQwBYtOVM/Dqp93pnLVGgD7EKLcmxWLF4g82Jx/JjSplNT19y0j14Z0Qp9TEpVe3mx51L86G0Yn30DAMDVQO5EzZUlRSEo4KxvNJCz/fpC+hfw7EZ92Yc0gF8HjfMBaKJaqtv+TpxLZMvNwE59vFnG4FyN6jLhmxOLq9Rgx1iCmiG/f7cmykDqcy5BVkzEPdVuWdRNCzhCari4Wrq4RJsRMYSCVDMEtu+Swwi0cSJe79tNBFQKC8QR9lhMEEAwNmMB+gDykH8m6J66DHLG5qb96IYbfDPTlc5PprjlTgFezzY6xuQrmyo1Dlw18AJ/H3HhJM5n2gijjcxKddTkXoo0= vasilysterekhov@nixos-laptop"
    "ssh-rsa AAAAB3NzaC1yc2EAAAADAQABAAABgQCj7VuKLTmExfro8brkb2EiZ4Ce+RWRILcZ24K4n+tfmOWuNFT9j07HNoTDXocMKYp4ycUxdAmHH1wtte8hPcovjPoUJQxDvNPGUWXgiFt7us6ngyNaOp4dRX5ViW4diwvbd5djhK2b5X05tWoLw2Z45mB81VxiM4yI0vSAIr4u/BDnf2SlcPJBM1sro94QDKjKL/zIiaHFPbgeBcWkL5Lm/tEmE5JsfkKBoUPJoEmDQxG5gtqi2p8d23SzPV53dqbTK6sNnpqrhmmxpuZ26gqqXa/LW3mjRE4Q/RD/BlSnVUQkAzp9fQvuNgIswclevUhGlgeDOA+dVF+XR98VPJWR2ylx0U7g3FUBnBIyM8/0BSu/XdVzFVLobtv2qZsWVTAH/KwD4K7Ktmr9HnxUeCbfCxAmz45PWawrAdlIHPmARuy/hhIi0c53FXrjWDW2fE99usMBSOT6ADnLGa7VlVSISDL3lL2ftQFu0TLAVkmqoFvNI1YRkYAi3ooFE5dNi3E= vasilysterekhov@nixos-desktop"
    "ssh-ed25519 AAAAC3NzaC1lZDI1NTE5AAAAIA8A0KqO5HOK7kzLjySfwYGWsrhDobeL3d1LcD2kksxP sholtokinghorn@gmail.com"
    # Bruce added 03/02 2025
    "ssh-rsa AAAAB3NzaC1yc2EAAAADAQABAAABgQCn/N9ssnsQlu7vL26n4KS3Mn8wz1dBI8SzK/1lj8IaeLfbjwu90F3wTTbgUVn7dU5mCyuobc5fHt8jkOEppn9r7sFcpxuKJhU6Y6q232TOEfW67Dw7uIAW4S5z2TTN4tzTCgr+qgbkmEqiDACNRacaixdcJd34oeNGh5JN7nmwj0izUyEmzDzquFxHyajdVDIy8/cYeQtOxqjREvF+0uo/3Xr0MLsfgKZ+o6n9tl1iaiI0jgGeIUkj4io1LsP3xgaYEcsZqCGGYI4tr+r/j6MFhJwUdMBiTydArZecIcKf3zFTIahHegP2sV8r82uXezF6BISgsRPeHsQj2NCgGQSkBHJuM/KRpjCGnbsZgyxUAl0763hYwGCMtskEsUUgLplXrVKwiriRgnY9S92YFuv3xCEnj2TnJD19UlUGpSx/EgBiwM9rgngizdSldGlPjHScvwVuN7L5HObycnPaHZihtQgGUgx0XMR8iS77cm9qlgDepRgJm0iclKmf+F7M4IM= ztq11@ztx"
    # Kian
    "ssh-ed25519 AAAAC3NzaC1lZDI1NTE5AAAAICE9odlcW4pwlwfPu2dfr6JRr4TUp56v2O+9sptC7Q7s kiandehmahdi3@gmail.com"
  ];

in {
  nix.settings.experimental-features = [ "nix-command" "flakes" ];

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

  hardware.bluetooth.enable = true;

  # https://wiki.nixos.org/wiki/NixOS_on_ARM/Raspberry_Pi_3#Bluetooth
  systemd.services.btattach = {
    before = [ "bluetooth.service" ];
    after = [ "dev-ttyAMA0.device" ];
    wantedBy = [ "multi-user.target" ];
    serviceConfig = {
      ExecStart =
        "${pkgs.bluez}/bin/btattach -B /dev/ttyAMA0 -P bcm -S 3000000";
    };
  };

  system.stateVersion = "24.05";

  nix.settings.trusted-public-keys = [
    "nixos-desktop:9+ZeV3IhjkppmYbFJPra5HG08ZmknrHIT/Fllz1h6SE="
    "nixos-laptop:FKieHCOcy6GVQkjgs+aZahQI2HYAYfq5NDtBVMiz8qY="
  ];

  users.users.pi = {
    isNormalUser = true;
    extraGroups = [ "wheel" ];
    openssh.authorizedKeys.keys = ssh-keys;
    password = "group13";
    linger = true;
  };

  home-manager.users.pi = { ... }: {
    imports = [ base-home ];
    home.username = "pi";
    home.homeDirectory = "/home/pi";
    home.stateVersion = "24.05";
    programs.home-manager.enable = true;
    home.file."DEMO/launch_demo.sh".source = ../DEMO/launch_demo.sh;
  };
}

