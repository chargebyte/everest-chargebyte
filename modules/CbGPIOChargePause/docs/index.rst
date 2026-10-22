.. SPDX-License-Identifier: Apache-2.0
.. Copyright 2026 chargebyte GmbH

CbGPIOChargePause
================

This module monitors one named GPIO input and calls ``set_external_limits``
on its required ``energy_sink`` interface. Connect it to a dedicated
EnergyNode so its limit remains independent of other limit sources.

When the input is logically active, the selected directions receive a 0 A
limit. When inactive, the module sends empty import, export, and setpoint
schedules to clear the external limits. EnergyNode then restores its
configured limits, including its fuse limit; constraints at other nodes
still apply.

Clearing replaces all external schedules at the connected node, rather than
removing entries by source. Use a dedicated EnergyNode for this module.

Configuration
-------------

``pause_gpio_line_name`` selects the input. ``pause_active_low: true`` means
a physical low level requests pausing; otherwise a high level does.
The line is held exclusively as an input with both-edge detection and
50 ms debounce, using the repository's libgpiod helper.

``pause_direction`` selects ``import`` (charging), ``export`` (discharging),
or ``both`` (the default). While paused, only the selected schedules are
populated. Setpoints and schedules for unselected directions are empty.
When released, all three schedules are empty regardless of this setting.

Operation and failures
----------------------

After framework readiness, the module sends the initial GPIO state and then
blocks waiting for edge events. It sends subsequent limits only when the
logical state changes. There is no periodic GPIO polling or limit refresh.
Schedules remain effective until replaced while the EnergyNode is running.
Restarting the EnergyNode separately requires restarting this module to
reapply an unchanged GPIO state.

GPIO acquisition and read failures are logged and retried after one second.
The last successfully applied limit is retained; before the first valid
reading no limit is sent. Failed command delivery is retried without marking
the state as applied. Recovery reads the current input rather than replaying
obsolete transitions.

Shutdown wakes and joins the monitoring worker, then releases its resources.
It does not clear or release the last energy limit. The shutdown member and
destructor use no framework-version-specific API, supporting both
stable/2026.02 and main.

Verification
------------

Enable ``CBGPIOCHARGEPAUSE_BUILD_TESTING`` and provide an installed GoogleTest
package to build the ``CbGPIOChargePause_tests`` target. Run its CTest entry
with ``ctest --test-dir build -R CbGPIOChargePause --output-on-failure``.

On target hardware, verify initial low/high levels with each polarity,
transitions in both directions, and 50 ms debounce. Capture calls to the
EnergyNode to check startup delivery, changes, and the absence of periodic
updates. Verify that stopping the module with an unchanged GPIO exits
promptly, and that a busy or unavailable GPIO retains the previous limit
and recovers when access becomes available.
