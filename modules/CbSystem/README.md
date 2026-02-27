# CbSystem Module

This module implements system wide operations.

Currently this includes the following commands:
*  Log Uploads
*  Firmware Updates
*  Setting of System time 

Corresponding variables signal the state of Log Uploads and Firmware Updates.

## Boot reason
To be able to notify the system of the finalized firmware update state (as
we rebooted before the update was successful), and that the boot reason was
a successful firmware update, we place a "marker" file into a file system
location which persists updates.  Its contents consist of the originally
inactive partition name (the one which receives the update), and the update
request ID.

On start-up of the module, existence of this marker file is checked.  If it
exists, a boot reason "FirmwareUpdate" is reported.  If we are running from
its named partition (now the active one), a firmware update status
"Installed" is reported.

Additionally, we have storage of the boot reason in a persistant store
module, which is optional.  This module can store the boot reason in a key
"ocpp_boot_reason" as "FirmwareUpdate" or "RemoteReset".  If the persistent
store is present, its value will be used for the boot reason, otherwise the
value "FirmwareUpdate" from the marker file handling.
