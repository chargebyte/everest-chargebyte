.. _everest_modules_handwritten_CbGPIOSMITH:

.. *******************************************
.. CbGPIOSMITH
.. *******************************************

Unlike the well-known STONITH (shoot the other node in the head) procedure in
high-availability setups, this module implements a "shoot myself in the head"
functionality - at least this is the base idea.

This function is required in some countries when a contactor fault (e.g. due
to welding) is detected as a second layer of safety.

This idea is here, to just trigger a GPIO output line which in turn ensures
in hardware that the power supply is turned off, a second in-series relais is
opened, or a circuit breaker is triggered... effectively "killing" ourself.

This module assumes for the moment, that this is a unavoidable one-way action,
i.e. the whole Linux system will be powered off instantaneously.
