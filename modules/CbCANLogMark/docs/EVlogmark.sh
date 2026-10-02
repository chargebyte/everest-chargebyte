#!/usr/bin/env bash

EVlogmark() {
    local can_interface=can0 can_id option payload
    local OPTIND=1

    while getopts ':d:' option; do
        case "$option" in
            d) can_interface=$OPTARG ;;
            *)
                echo "Usage: EVlogmark [-d CAN_INTERFACE] {error|warning|info|debug} MARKER" >&2
                return 2
                ;;
        esac
    done
    shift $((OPTIND - 1))

    if [ "$#" -ne 2 ]; then
        echo "Usage: EVlogmark [-d CAN_INTERFACE] {error|warning|info|debug} MARKER" >&2
        return 2
    fi

    case "$1" in
        error)   can_id=6F0 ;;
        warning) can_id=6F1 ;;
        info)    can_id=6F2 ;;
        debug)   can_id=6F3 ;;
        *)
            echo "Usage: EVlogmark [-d CAN_INTERFACE] {error|warning|info|debug} MARKER" >&2
            return 2
            ;;
    esac

    payload="$({ printf '%s' "$2"; printf '\0\0\0\0\0\0\0\0'; } | head -c 8 | xxd -p -c 8)"
    cansend "$can_interface" "${can_id}#${payload}"
}

if [[ ${BASH_SOURCE[0]} == "$0" ]]; then
    EVlogmark "$@"
fi
