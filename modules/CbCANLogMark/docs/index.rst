.. _everest_modules_handwritten_CbCANLogMark:

.. *******************************************
.. CbCANLogMark
.. *******************************************

``CbCANLogMark`` listens for fixed-size text markers on a Classic CAN interface
and forwards them to the EVerest logger. It does not transmit CAN frames.

The default 11-bit CAN IDs are:

* ``0x6F0`` for ``EVLOG_error``
* ``0x6F1`` for ``EVLOG_warning``
* ``0x6F2`` for ``EVLOG_info``
* ``0x6F3`` for ``EVLOG_debug``

All IDs can be changed in the module configuration. They must be unique. Only
frames with a DLC of eight are accepted. The payload is interpreted as a byte
string and ends at the first NUL byte; when no NUL is present, all eight bytes
are logged.

The accompanying ``CbCANLogMark.dbc`` describes the default IDs. If IDs are
overridden in the EVerest configuration, the DBC needs to be adjusted for tools
that use it.

Sending a marker with ``cansend``
=================================

``cansend`` expects the CAN payload in hexadecimal notation. ``xxd`` can be
used to convert a text marker on the command line. For example, the following
command sends ``Stamp1`` with info severity on ``can0``::

    cansend can0 "6F2#$(printf 'Stamp1\0\0' | xxd -p -c 8)"

The two NUL bytes pad the marker to the required payload size of eight bytes.
For repeated use, the accompanying :download:`EVlogmark.sh <EVlogmark.sh>`
provides the ``EVlogmark`` Bash function. Source the script into the current
shell and call the function with a severity and a marker::

    source EVlogmark.sh
    EVlogmark info Stamp1

Alternatively, the script can be executed directly::

    ./EVlogmark.sh info Stamp1

Quote the second argument if the marker contains spaces, for example
``EVlogmark warning "Hot plug"``.

The helper sends on ``can0`` by default. Use ``-d`` before the severity to
select a different CAN interface::

    EVlogmark -d can1 info Stamp1

The same option is available when executing the script directly::

    ./EVlogmark.sh -d can1 debug Stamp1

The helper uses the default CAN IDs shown above. Adjust the values in the
``case`` statement if the module is configured with different IDs. A marker is
limited to eight bytes, not necessarily eight characters when a multibyte
encoding such as UTF-8 is used.
