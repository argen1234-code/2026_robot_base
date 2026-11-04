#!/bin/bash
# Expose the Orbbec Astra Pro RGB (UVC) nodes inside a privileged container
# that was started before the camera was plugged in.
#
# Run inside the container:
#   bash src/perception/scripts/setup_orbbec_uvc.sh
#
# The major/minor numbers are read from sysfs, so they survive re-enumeration.
set -e

for idx in 0 1; do
    sys_dev="/sys/class/video4linux/video${idx}/dev"
    node="/dev/video${idx}"
    [ -e "$sys_dev" ] || continue
    dev=$(cat "$sys_dev")          # e.g. "81:17"
    major=${dev%%:*}
    minor=${dev##*:}
    if [ ! -e "$node" ]; then
        mknod "$node" c "$major" "$minor"
    fi
    chmod 660 "$node" 2>/dev/null || true
    chown root:video "$node" 2>/dev/null || true
    echo "ready: $node ($major:$minor)"
done
