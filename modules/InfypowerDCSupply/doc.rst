.. _everest_modules_handwritten_InfypowerDCSupply:

*****************
InfypowerDCSupply
*****************

Usage Overview
==============

This module is a driver for various Infypower's DC supply power modules
connected via CAN bus to the EVerest system.

It provides the `power_supply_DC` interface and there are no interface dependencies.

The manifest has reasonable default values set, e.g. for the CAN interface
and the bitrate, but also for the Infypower addresses used in the CAN IDs.
If required, it can be adapted for your setup.

The idea of this implementation is, that different power modules are grouped
together to form a single "virtual" power module with combined power.
For this, the power modules must be physically configured correctly, e.g.
the group identification number must be manually configured on each module.
See Infypower's user manuals for details.

It is necessary to configure the power module type, or in other words the
series of the power module, since this cannot be detected automatically
via CAN (or at least not easily).

Implementation Overview
=======================

Beyond the usual EVerest framework class files, the source code consists of
additional C++ source files with several C++ classes. The idea is to model
various aspects of Infypower's CAN communication protocol into dedicated
C++ classes so that it is easier and faster to adapt the code for new models
and/or changed requirements.

The used CAN IDs for example, include a source and a destination address
which must be swapped for example when sending vs. receiving a command.
They include also error codes which must be considered when receiving. For this,
the helper classes provide e.g. helpers for masking out the specific bits in
the CAN IDs.

From the high-level perspective, the protocol is a command-response approach.
There are commands which need to be sent in a cyclic manner, e.g. querying
the current voltage and current or the power module status.
Other commands only need to be sent once, or in case a specific action should
be done, like changing the working mode or adjusting voltage/current.
Each command usually causes a response from the power module. However, there
are commands when only the master module of the group answers, but there are
also commands when all power modules of the group respond with a CAN frame.

For cyclic CAN frames, the Linux kernel offers the CAN BCM interface.
This is used to generate the cyclic CAN query frames.
It also offers a nice filter solution when receiving frames, but this can only
be used for the status messages. The reason is that EVerest expects the current
voltage and current reported periodically, so the filtering of the BCM would
suppress too many frames and additional user-space logic would be required.
So the decision was made, that those frames are received using Linux' CAN RAW
interface.

This raw interface is better suited for sending out the non-cyclic CAN frames,
but also provides flexible CAN ID based filters while receiving - in contrast to
the BCM interface which provides filter based on content changes.
This is why also such a socket is used.

The command class tries to abstract the process of sending out the query and
sleeping until one or all response frames are received. For this, it uses
a condition variable on which the EVerest interface handling thread sleeps
and the background thread of receiving CAN frames signals each received CAN frame.
In case of lost CAN frames, the sleeping is coupled with a timeout.
Because the cyclic CAN frames do not have a dedicated count of expected response
frames, a callback method is implemented as additional approach of handling
responses.

Since different power modules use different commands, the idea is to use the
existing controller class as base class later, and derive power module specific
sub-classes. These classes can then implement specific functionality while
providing the same interface to higher layers.

Open TODOs/Ideas
================

* test bidirectional charging with BEC/BEG power modules
  (only usual AC -> DC was tested with a BEG power module at the moment)
* rework controller class so that sub-classes can better handle
  different power module types (with slightly different commands
  and/or datatypes etc.)
* rework the class for command handling, e.g. byte0/byte1 stuff
  is not required for some commands
* improve the alarm handling code
* clarify how to handle fields like peak_current_ripple_A... and what is
  the source of these values
