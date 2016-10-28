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

from numpy import zeros

from simulation import Process
from threading import Lock


class PreviewBuffer(Process):

    """
    Store preview trajectories and execute them until the next update.
    """

    def __init__(self, com, free_foot):
        super(PreviewBuffer, self).__init__()
        self.com = com
        self.com_traj = None
        self.comdd = zeros(3)
        self.foot_traj = None
        self.free_foot = free_foot
        self.lock = Lock()
        self.preview_time = 0.
        self.rem_time = 0.

    @property
    def is_empty(self):
        return self.com_traj is None

    def update_preview(self, com_traj, foot_traj):
        """
        Update preview trajectories.
        """
        with self.lock:
            self.com_traj = com_traj
            self.foot_traj = foot_traj
            self.preview_time = 0.
        self.comdd = self.com_traj.Evaldd(0.)

    def on_tick(self, sim):
        if self.is_empty:
            return
        with self.lock:
            if self.preview_time < sim.dt:
                comdd = self.com_traj.comdd_beg  # this one is carefully checked
            else:  # if no other choice, use stored trajectory
                comdd = self.com_traj.Evaldd(self.preview_time)
            self.comdd = comdd
            self.com.integrate_acceleration(comdd, sim.dt)
            self.preview_time += sim.dt
            if self.foot_traj is not None:
                t = min(self.preview_time, self.foot_traj.duration)
                s = self.foot_traj.s_traj.Eval(t)
                footd = self.foot_traj.Evald(t)
                foot = self.foot_traj.Eval(t)
                self.free_foot.set_velocity(footd)
                self.free_foot.set_pos(foot)
                self.free_foot.update_orientation(s)
