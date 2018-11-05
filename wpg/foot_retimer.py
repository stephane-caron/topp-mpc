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

from numpy import ones

from TOPP.TOPPbindings import TOPPInstance
from TOPP.Utilities import vect2str
from hermite import interpolate_houba_topp
from pymanoid.misc import norm
from pymanoid.rotations import rotation_matrix_from_quat
from retimer import BodyRetimer


class FootRetimer(BodyRetimer):

    def __init__(self, foot, max_foot_accel=5):
        """
        Create a new foot retimer.

        INPUT:

        - ``foot`` -- FreeFoot instance
        - ``max_foot_accel`` -- maximum foot acceleration, in [m] / [s]^2
        """
        super(FootRetimer, self).__init__(foot)
        self.max_foot_accel = max_foot_accel

    def interpolate_path(self):
        target_pose = self.body.end_pose
        target_p = target_pose[4:]
        R = rotation_matrix_from_quat(target_pose[:4])
        t, n = R[0:3, 0], R[0:3, 2]
        target_pd = 0.5 * t - 0.5 * n
        p0 = self.body.p
        p1 = target_p
        if norm(p1 - p0) < 1e-3:
            return None, None
        v1 = target_pd
        if norm(self.body.pd) > 1e-4:
            v0 = self.body.pd
            path = interpolate_houba_topp(p0, p1, v0, v1)
            sd_beg = norm(v0) / norm(path.Evald(0.))
        else:  # choose initial direction
            v0 = 0.3 * self.body.t + 0.7 * self.body.n
            path = interpolate_houba_topp(p0, p1, v0, v1)
            sd_beg = 0.
        return path, sd_beg

    def compute_topp_instance(self, path, sd_beg):
        amax = self.max_foot_accel * ones(path.dimension)
        id_traj = "1.0\n1\n0.0 1.0"
        discrtimestep = self.discrtimestep
        ndiscrsteps = int((path.duration + 1e-10) / discrtimestep) + 1
        constraints = str(discrtimestep)
        constraints += "\n" + str(0.)  # no velocity limit
        for i in range(ndiscrsteps):
            s = i * discrtimestep
            ps = path.Evald(s)
            pss = path.Evaldd(s)
            constraints += "\n" + vect2str(+ps) + " " + vect2str(-ps)
            constraints += "\n" + vect2str(+pss) + " " + vect2str(-pss)
            constraints += "\n" + vect2str(-amax) + " " + vect2str(-amax)
        inst = TOPPInstance(None, "QuadraticConstraints", constraints, id_traj)
        inst.integrationtimestep = 1e-3
        return inst
