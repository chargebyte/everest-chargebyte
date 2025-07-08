.. _everest_modules_handwritten_CbChargeSOMDriver:

*****************
CbChargeSOMDriver
*****************

Usage Overview
==============

This module is a BSP driver for chargebyte's Charge SOM platform.
It provides the `evse_board_support` interface and an additional custom
interface for temperature monitoring.

The manifest has reasonable default values set for the intended default
usage of DC charging. However, it is recommended to double-check the
identifiers for naming the PT1000 channels.

The module assumes, that the safety controller is already pre-configured
with a parameter block which matches the desired target setup.

Implementation Overview
=======================

The Charge SOM platform uses a UART connection between the EVerest system and
the safety controller. The protocol used there is documented in the User Guide:
https://chargebyte-docs.readthedocs.io/projects/everest-charge-som

For the low-level UART communication part, a library is used which can be
found here: https://github.com/chargebyte/ra-utils
This tool can be used for debugging and development purposes and thus the
already written code was just re-used here.

On level above this library, the C++ class `CbChargeSOM` exists which abstracts
the C library handling into a C++ interface. This class also tries to act as
a bridge between the required periodic communication and the event-based EVerest
system. For this, there are 3 threads:
- RX thread: This thread just cares about receiving the UART frames sent by
  the safety controller firmware. There are 3 different "classes" of frame and
  handling is different for each class. First, the Charge State frame is sent
  in a very short time frame periodically. This frame includes most of the
  data points which the EVerest system has to look at. But it is not possible
  to react immediately within this thread since processing might take more time
  and this could potentially result in loosing frames. So the RX thread
  compares the previous message with the just received one and in case there is
  a difference, it puts the new value into a queue for later processing.
  Then there is the PT1000 temperature frame. It is also sent by the safety
  controller in a periodic, very short interval. But here, the processing can
  be done asynchronously. The RX thread just stores the received data.
  The third class of frames are the ones which are only received upon request
  by an inquiry frame. Usually, the requesting function wants to be informed
  about the incoming frame, thus a notify mechanism is used based on condition
  variables.
- Notify thread: This thread does the main work for the EVerest `evse_board_support`
  interface. It checks the individual data points for changes and invokes
  several slots. The callback functions of these slots then process the actual
  change, e.g. inform EVerest system with matching/expected data structures etc.
- TX thread: This thread is responsible for periodically sending out the Charge Control
  frame. While it is possible for `evse_board_support` callback handlers to trigger
  sending a Charge Control frame immediately, this thread just acts as a background
  worker to satisfy the safety controller with periodic frames - otherwise the
  safety controller would go into a safe state.

On the next level, there are the already mentioned EVerest interfaces `evse_board_support`
and `temperatures`. The latter implements its own thread which pulls the temperatures
in a one second interval from the (cache) structures of `CbChargeSOM` and publishes
them together with a descriptive identifier string. This interface is not
related to the whole charging process, it just allows monitoring the temperature values.

Open TODOs/Ideas
================

* no Type 1 support yet
* contactor handling is very simple at the moment
