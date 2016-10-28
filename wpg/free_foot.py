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

from numpy import array, zeros

from pymanoid import Contact
from pymanoid.rotations import quat_slerp


class FreeFoot(Contact):

    def __init__(self, **kwargs):
        super(FreeFoot, self).__init__(X=0.2, Y=0.1, **kwargs)
        self.__pd = zeros(3)
        self.end_pose = None
        self.start_pose = None

    def set_velocity(self, pd):
        """Update the point-mass velocity."""
        self.__pd = array(pd)

    @property
    def pd(self):
        return self.__pd.copy()

    def reset(self, start_pose, end_pose):
        self.__pd *= 0.
        self.end_pose = end_pose
        self.set_pose(start_pose)
        self.start_pose = start_pose

    def update_orientation(self, s):
        """SLERP interpolation of foot orientation (0. <= s <= 1.)."""
        self.set_quat(quat_slerp(self.quat, self.end_pose[0:4], s))
