.. _nsim:

DesignWare(R) ARC(R) Emulation (nsim)
#####################################

Overview
********

This board configuration will use `Designware ARC nSIM`_ to emulate a virtual
ARC EM or ARC HS based board including the following features:

* ARC EM or ARC HS processor
* ARC internal timer
* a virtual output only console (uart-nsim)

There are three supported board sub-configurations:

* ``nsim_em`` which includes normal ARC EM features and ARC MPUv2
* ``nsim_sem`` which includes secure ARC EM features and ARC MPUv3
* ``nsim_hs`` which includes base ARC HS features, i.e. w/o PMU and MMU

For detailed arc features, please refer to
:zephyr_file:`boards/arc/nsim/support/nsim_em.props`,
:zephyr_file:`boards/arc/nsim/support/nsim_sem.props` and
:zephyr_file:`boards/arc/nsim/support/nsim_hs.props`.


Hardware
********
Supported Features
==================

The following hardware features are supported:

+-----------+------------+-----+-------+-----+-----------------------+
| Interface | Controller | EM  | SEM   | HS  | Driver/Component      |
+===========+============+=====+=======+=====+=======================+
| INT       | on-chip    | Y   | Y     | Y   | interrupt_controller  |
+-----------+------------+-----+-------+-----+-----------------------+
| UART      | nsim uart  | Y   | Y     | Y   | serial port-polling   |
+-----------+------------+-----+-------+-----+-----------------------+
| TIMER     | on-chip    | Y   | Y     | Y   | system clock          |
+-----------+------------+-----+-------+-----+-----------------------+


Programming and Debugging
*************************

Required Hardware and Software
==============================

To use Zephyr RTOS applications on this board, `Designware ARC nSIM`_ or
`Designware ARC nSIM Lite`_ is required.

Building Sample Applications
==============================

Use this configuration to run basic Zephyr applications and kernel tests in
nSIM, for example, with the :ref:`synchronization_sample`:

.. zephyr-app-commands::
   :zephyr-app: samples/synchronization
   :host-os: unix
   :board: nsim_em
   :goals: flash

This will build an image with the synchronization sample app, boot it using
nsim, and display the following console output:

.. code-block:: console

        ***** BOOTING ZEPHYR OS v1.12 - BUILD: July 6 2018 15:17:26 *****
        threadA: Hello World from arc!
        threadB: Hello World from arc!
        threadA: Hello World from arc!
        threadB: Hello World from arc!


.. note::
   To exit the simulator, use Ctrl+], then Ctrl+c

Debugging
=========

Refer to the detailed overview about :ref:`application_debugging`.

References
**********

.. _Designware ARC nSIM: https://www.synopsys.com/dw/ipdir.php?ds=sim_nsim
.. _Designware ARC nSIM Lite: https://www.synopsys.com/cgi-bin/dwarcnsim/req1.cgi
