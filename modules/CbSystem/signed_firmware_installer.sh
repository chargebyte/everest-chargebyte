#!/bin/bash

. "${1}"

echo "$INSTALLING"
sleep 2

# Install the firmware image
rauc install "${2}" >/dev/null 2>&1

# Get the output of "rauc status" command
status_output=$(rauc status)

# Get the line containing "Booted from" and extract the value after the colon
booted_from=$(echo "$status_output" | grep "Booted from" | awk -F ': ' '{print $2}')

# Get the line containing "Activated" and extract the value after the colon
activated=$(echo "$status_output" | grep "Activated" | awk -F ': ' '{print $2}')

# Get the number of partitions marked as good
marked_good=$(echo "$status_output" | grep "boot status" | grep -c good)

# Check if booted_from and activated are different and the 2 partitions are marked as good
if [[ "$booted_from" != "$activated" && "$marked_good" -eq 2 ]]; then
    echo "$INSTALL_REBOOTING"
else
    echo "$INSTALLATION_FAILED"
fi
