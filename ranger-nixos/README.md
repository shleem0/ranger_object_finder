# ranger-nixos

NixOS configuration for Ranger. _Will_ automatically include all of our packages
and their dependencies, currently just builds `ranger-daemon`.

To apply the configuration on the Raspberry Pi, NixOS must also be installed on your
computer (whether WSL, VM, or native, doesn't matter). But if you only need to run
a VM with the Raspberry Pi's settings, you just need Nix (the package manager): see
`nixosConfigurations.sdp-local` below.

Configurations are specified in the top-level `flake.nix`.

To build ROS packages to run on the Raspberry Pi as part of the configuration,
see https://github.com/lopsided98/nix-ros-overlay

Password login over SSH is disabled. SSH keys are stored in `sdp.nix`: add your public key
there to be able to SSH into the Raspberry Pi.
You can also log in by directly connecting a monitor and keyboard. The username
is `pi` and the password is `group13`.

## Notable pre-installed packages

- [Distrobox](https://github.com/89luca89/distrobox) if you need Ubuntu for development

- [Tailscale](https://tailscale.com) to SSH into the Raspberry Pi from any network as
  long as it's connected to the internet.

- [fish](https://fishshell.com), a somewhat nicer shell than Bash (has
  syntax highlighting, better autocomplete and command hints) (you can still run
  bash by explicitly calling `bash`, but there is no reason to)

- `ranger-daemon` (stub)

- `ranger_find_item_in_scene`: From ranger_object_recognition: see its own README.
  Script in PATH taking reference images and a scene image, returning object
  recognition result.

## Configurations

### `nixosConfigurations.sdp`

Intended configuration for the Raspberry Pi.

Includes `ranger-daemon`, plus extra settings like networking, authorised SSH keys,
fish shell, Distrobox, Tailscale, etc.

The only initial setup necessary after flashing a fresh SD image is enabling
Tailscale with

```
tailscale up
```

and logging in.

Currently, it will automatically connect to any hotspot with the name `ranger-hotspot`
and password `12345678`. Though I recommend using wired tethering instead to
connect the Raspberry Pi to the internet, as it fails to connect to mobile
hotspots whenever it is connected to a monitor over HDMI (no problems when
disconnected). I have no explanation for why this happens.

### `nixosConfigurations.sdp-local`

Mostly identical to the `sdp` configuration, except excludes some Raspberry
Pi-specific settings and builds for `x86_64-linux`. `x86_64` does not have
the problem with caches, so this configuration includes `ranger-daemon`.

You can run a VM with this configuration with the command 

```bash
nix run .#nixosConfigurations.sdp-local.config.system.build.nixos-shell
```

(delete the `qcow2` file after, otherwise it breaks)

## Setup

There are several options to apply this configuration on a Raspberry Pi.

As mentioned above, if you are applying `sdp`: in all cases, the first run will
take ages, especially if flashing a completely new SD image

Subsequent runs will take a lot less time.

You will need these lines in your NixOS configuration:

```nix
nix.settings.experimental-features = [ "nix-command" "flakes" ];

boot.binfmt.emulatedSystems = [ "aarch64-linux" ];

boot.binfmt.preferStaticEmulators = true;
```

### [`deploy-rs`](https://github.com/serokell/deploy-rs)

The simplest option if you can SSH into the Raspberry Pi.

```bash
nix shell nixpkgs#deploy-rs

# IP address of the Raspberry Pi, or just sdp-ranger if you have Tailscale set up
# and the Raspberry Pi is connected to the internet.
RASPBERRY_PI_HOSTNAME=sdp-ranger

deploy .#sdp --hostname "$RASPBERRY_PI_HOSTNAME"
```

This will automatically roll the configuration back if the Raspberry Pi loses
connectivity from the new configuration.

### Flashing a new SD image

Completely fresh install. Not recommended unless there is no other option (like if
you lose the SD card)

Build and extract the image:

```bash
# builds the configuration, will take a long time
nix build .#nixosConfigurations.sdp.config.system.build.sdImage

# this may also take 20-30 minutes
nix shell nixpkgs#zstd -c unzstd -o result.img ./result/sd-image/*.img.zst
```

Find the SD card's device file using `lsblk`:

```bash
lsblk

# look at command output...

# IMPORTANT: be absolutely sure that the path actually points to the SD card,
# this will destroy all data on the disk
SD_CARD_FILE=/dev/path/to/sd/card # such as /dev/sdb
```

Flash the image on the SD card:

```bash
# and this may also take 15ish minutes
sudo dd if=result.img of=$SD_CARD_FILE status=progress bs=4M
```

Once the last command exits, you are done and can eject and remove the SD card.

### `nixos-enter`

If you have access to the SD card, but the Raspberry Pi cannot connect to a network.

Copy this repository somewhere on the SD card, then `cd` into the SD card's
`NIXOS_SD` partition, then run

```bash
sudo nixos-enter --root .

nixos-rebuild switch --flake /path/to/ranger_object_finder#nixosConfigurations.sdp --option sandbox false --option filter-syscalls false
```

Once it's built, you are done and can eject and remove the SD card.
