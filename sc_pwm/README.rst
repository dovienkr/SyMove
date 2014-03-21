XCORE Multi Channel PWM SOFTWARE COMPONENT
..............................................
.. image:: http://forum.synapticon.com/Themes/MinimalistAndEffective_by_SMFSimple/images/logo.png
.. image:: http://s27.postimg.org/higfoxmn7/xmos_logo_reduced.png

The Pulse Width Modulation(PWM) components generates a number PWM signals using either one multibit port or a group of 1-bit ports. 

:Latest release: 1.0.0rc0
:Maintainer: djpwilk
:Maintainer: Gopal Lakshmanagowda (github: nlgk2001)
:Maintainer: support@synapticon.com

Key Features
============

  * The components can be configured for Leading edge, Trailing edge and Center edge variations
  * Configurable timestep, resolution
  * PWM single bit component generates PWM signals on upto 16 1-bit ports from a single thread.
  * PWM multi bit component generates the PWM signals on a single 4, 8 or 16 bit port.

Firmware Overview
=================

The components will run in a par with the following function which does not terminate. A single function starts the pwm server and passes it a channel with 
which it will communicate with the client, a clock block required for the clocking of the required ports, an array of ports on which the pwm signals will be generated, and the number of ports in the array. 

Known Issues
============

none

Required software (dependencies)
================================

  * None

Changelog
=======

- 1.0.0

  * Initial release

License
=======

Please see `LICENSE`_.


.. _LICENSE: https://github.com/synapticon/sc_pwm/blob/master/LICENSE.dox
