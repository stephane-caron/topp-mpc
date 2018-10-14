# TOPP-MPC

Source code for https://hal.archives-ouvertes.fr/hal-01363757/document

## Installation

On Ubuntu 14.04, you will need to [install
OpenRAVE](https://scaron.info/teaching/installing-openrave-on-ubuntu-14.04.html)
and [TOPP](https://github.com/quangounet/TOPP). Then, clone this repository and 
its submodule via:
```bash
git clone --recursive https://github.com/stephane-caron/topp-mpc.git
```
If you already have [pymanoid](https://github.com/stephane-caron/pymanoid) installed on your system, make sure to run the main script from the project folder directly (so that it uses the local rather than system version of pymanoid).

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
