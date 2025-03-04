{ ... }:

{
  networking.wireless.enable = true;

  # Allow using /etc/wpa_supplicant.conf
  networking.wireless.allowAuxiliaryImperativeNetworks = true;

  # Use SSH machine host keys for decryption
  sops.age.sshKeyPaths = [ "/etc/ssh/ssh_host_ed25519_key" ];

  # Encrypted wifi config
  sops.secrets."wpa_supplicant.conf" = {
    format = "binary";
    sopsFile = ./secrets/wpa_supplicant.conf;
    path = "/etc/wpa_supplicant.conf";
  };

  # fallback if wpa_supplicant.conf doesn't work
  networking.wireless.networks."ranger-hotspot" = {
    psk = "12345678";
    priority = -1;
  };
}
