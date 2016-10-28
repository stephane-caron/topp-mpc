#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# Copyright (C) 2016 Stephane Caron <stephane.caron@normalesup.org>
#
# This file is part of topp-mpc <https://github.com/stephane-caron/topp-mpc>.
#
# topp-mpc is free software: you can redistribute it and/or modify it under
# the terms of the GNU General Public License as published by the Free Software
# Foundation, either version 3 of the License, or (at your option) any later
# version.
#
# topp-mpc is distributed in the hope that it will be useful, but WITHOUT
# ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
# FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more
# details.
#
# You should have received a copy of the GNU General Public License along with
# topp-mpc. If not, see <http://www.gnu.org/licenses/>.

from com_retimer import COMRetimer
from foot_retimer import FootRetimer
from simulation import Process


class TOPPPreviewControl(Process):

    def __init__(self, com, foot, fsm, preview_buffer):
        """
        Create a new feedback controller that continuously runs the preview
        controller and sends outputs to a COMAccelBuffer.

        INPUT:

        - ``com`` -- PointMass containing current COM state
        - ``foot`` -- FreeFoot used in swinging phases
        - ``fsm`` -- finite state machine
        - ``preview_buffer`` -- PreviewBuffer to send MPC outputs to
        """
        super(TOPPPreviewControl, self).__init__()
        self.com = com
        self.com_retimer = COMRetimer(com, fsm)
        self.fsm = fsm
        self.foot = foot
        self.foot_retimer = FootRetimer(foot)
        self.preview_buffer = preview_buffer
        self.thread = None
        self.thread_lock = None
        self.verbose = False

    def on_tick(self, sim):
        """
        Compute a new control after a tick of the control loop.

        INPUT:

        - ``sim`` -- instance of current simulation
        """
        if self.fsm.cur_stance.is_single_support:
            self.foot_retimer.update()
            foot_traj = self.foot_retimer.get_trajectory()
            com_succ = self.com_retimer.update(foot_traj.duration)
        else:  # cur stance is double support
            foot_traj = None
            com_succ = self.com_retimer.update()
        if com_succ:
            com_traj = self.com_retimer.get_trajectory()
            self.preview_buffer.update_preview(com_traj, foot_traj)

    def plot_profiles(self):
        import pylab
        pylab.subplot(121)
        self.com_retimer.plot_profiles()
        pylab.subplot(122)
        self.foot_retimer.plot_profiles()
