#!/bin/bash

. "${1}"

mkdir /tmp/signature_validation
sleep 2
echo "$DOWNLOADING"

sleep 2
curl --location --progress-bar --fail --connect-timeout "$CONNECTION_TIMEOUT" "${2}" -o "${3}"
curl_exit_code=$?
sleep 2
if [[ $curl_exit_code -eq 0 ]]; then
    echo "$DOWNLOADED"
    echo -e "${4}" >/tmp/signature_validation/firmware_signature.base64
    echo -e "${5}" >/tmp/signature_validation/firmware_cert.pem
    openssl x509 -pubkey -noout -in /tmp/signature_validation/firmware_cert.pem >/tmp/signature_validation/pubkey.pem # Extract public key from certificate
    openssl base64 -d -in /tmp/signature_validation/firmware_signature.base64 -out /tmp/signature_validation/firmware_signature.sha256 # Decode base64 signature to binary
    r=$(openssl dgst -sha256 -verify /tmp/signature_validation/pubkey.pem -signature /tmp/signature_validation/firmware_signature.sha256 -sigopt rsa_padding_mode:pss -sigopt rsa_pss_saltlen:-1 "${3}") # Verify signature
    if [ "$r" = "Verified OK" ]; then
        echo "$SIGNATURE_VERIFIED"
    else
        echo "$INVALID_SIGNATURE"
    fi
else
    echo "$DOWNLOAD_FAILED"
fi

rm -rf /tmp/signature_validation
