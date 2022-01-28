TARGETS = console-setup alsa-utils mountkernfs.sh ufw pppd-dns hostname.sh plymouth-log dns-clean x11-common apparmor udev mountdevsubfs.sh resolvconf procps brltty hwclock.sh networking urandom checkroot.sh checkroot-bootclean.sh bootmisc.sh mountnfs-bootclean.sh mountnfs.sh kmod mountall-bootclean.sh mountall.sh checkfs.sh
INTERACTIVE = console-setup udev checkroot.sh checkfs.sh
udev: mountkernfs.sh
mountdevsubfs.sh: mountkernfs.sh udev
resolvconf: dns-clean
procps: mountkernfs.sh udev
brltty: mountkernfs.sh udev
hwclock.sh: mountdevsubfs.sh
networking: mountkernfs.sh urandom resolvconf procps dns-clean
urandom: hwclock.sh
checkroot.sh: hwclock.sh mountdevsubfs.sh hostname.sh
checkroot-bootclean.sh: checkroot.sh
bootmisc.sh: checkroot-bootclean.sh mountnfs-bootclean.sh mountall-bootclean.sh udev
mountnfs-bootclean.sh: mountnfs.sh
mountnfs.sh: networking
kmod: checkroot.sh
mountall-bootclean.sh: mountall.sh
mountall.sh: checkfs.sh checkroot-bootclean.sh
checkfs.sh: checkroot.sh
