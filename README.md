# TOPP-MPC

Source code for https://hal.archives-ouvertes.fr/hal-01363757/document

## Installation

On Ubuntu 14.04, you will need to [install
OpenRAVE](https://scaron.info/teaching/installing-openrave-on-ubuntu-14.04.html)
and [TOPP](https://github.com/quangounet/TOPP).

## Usage

Run the main script ``./walk.py``. Then, you can

- start simulations by typing ``sim.start()`` in the Python prompt, or
- use ``sim.step(n)`` to run simulations in stepping mode for ``n`` steps.

The state of all objects can be inspected using the global variables ``robot``,
``fsm`` (state machine) and ``mpc`` (preview controller).

### Robot model

Due to the copyright problem, we cannot release the COLLADA model ``HRP4R.dae``
used to produce the accompanying video and paper illustrations. It is replaced
at run time by
[JVRC-1](https://github.com/stephane-caron/openrave_models/tree/master/JVRC-1),
which has the same kinematic chain.
