#!/bin/bash

. "${1}"

echo "$DOWNLOADING"
curl --progress-bar --connect-timeout "$CONNECTION_TIMEOUT" "${2}" -o "${3}"
curl_exit_code=$?
sleep 2

if [[ $curl_exit_code -eq 0 ]]; then
    echo "$DOWNLOADED"
else
    echo "$DOWNLOAD_FAILED"
fi
sleep 2

if [[ $curl_exit_code -eq 0 ]]; then
    echo "$INSTALLING"
    rauc install "${3}" >/dev/null 2>&1
fi

# Get the line containing "Booted from" and extract the value after the colon
booted_from=$(rauc status | grep "Booted from" | awk -F ': ' '{print $2}')

# Get the line containing "Activated" and extract the value after the colon
activated=$(rauc status | grep "Activated" | awk -F ': ' '{print $2}')

# Get the number the number of paritions marked as good
marked_good=$(rauc status | grep "boot status" | grep -c good)

# Check if booted_from and activated are different and the 2 paritions are marked as good
if [[ "$booted_from" != "$activated" && "$marked_good" -eq 2 ]]; then

    echo "$INSTALL_REBOOTING"
else
    echo "$INSTALLATION_FAILED"
fi
