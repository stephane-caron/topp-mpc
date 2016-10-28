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

import os

from numpy import dot

from pymanoid import ContactSet
from pymanoid.polyhedra import Polytope

if os.path.isfile('HRP4R.dae'):
    from pymanoid.robots import HRP4 as RobotModel
else:
    from pymanoid.robots import JVRC1 as RobotModel


class Stance(ContactSet):

    def __init__(self, phase, left_foot=None, right_foot=None,
                 ref_velocity=0.4):
        """
        Create a new stance.

        INPUT:

        - ``phase`` -- corresponding phase in the FSM
        - ``left_foot`` -- (optional) left foot contact
        - ``right_foot`` -- (optional) right foot contact
        - ``ref_velocity`` -- (default: 0.4 m/s) target forward COM velocity
        """
        contacts = {}
        if left_foot:
            contacts['left_foot'] = left_foot
        if right_foot:
            contacts['right_foot'] = right_foot
        lateral_offset = 0.01
        if phase[-1] == 'L':
            target_foot = left_foot
            com = left_foot.p + [0., 0., RobotModel.leg_length]
            com -= lateral_offset * left_foot.b
        else:  # phase[-1] == 'R'
            target_foot = right_foot
            com = right_foot.p + [0., 0., RobotModel.leg_length]
            com += lateral_offset * right_foot.b
        self.com = com
        self.comd = ref_velocity * target_foot.t
        self.is_double_support = phase.startswith('DS')
        self.is_single_support = phase.startswith('SS')
        self.left_foot = left_foot
        self.right_foot = right_foot
        self.target_foot = target_foot
        self.phase = phase
        super(Stance, self).__init__(contacts)
        self.compute_stability_criteria()

    def compute_stability_criteria(self):
        def update_contacts(X, Y, mu):
            for contact in self.contacts:
                contact.X = X
                contact.Y = Y
                contact.friction = mu

        X = self.contact_dict.values()[0].X
        Y = self.contact_dict.values()[0].Y
        mu = self.contact_dict.values()[0].friction
        topp_scale = 0.75
        update_contacts(topp_scale * X, topp_scale * Y, topp_scale * mu)
        self.cwc_topp = self.compute_wrench_cone([0, 0, 0])
        update_contacts(X, Y, mu)

        self.sep = self.compute_static_equilibrium_polygon()
        self.sep_hrep = Polytope.hrep(self.sep)

    def dist_to_sep_edge(self, com):
        """
        Algebraic distance to the edge of the static-equilibrium polygon
        (positive inside, negative outside).
        """
        A, b = self.sep_hrep
        return min(b - dot(A, com[:2]))

    def is_inside_sep(self, com):
        """True if and only if ``com`` is strictly inside the SEP."""
        return self.dist_to_sep_edge(com) > 0

    def is_outside_sep(self, com):
        """True if and only if ``com`` is strictly outside the SEP."""
        return self.dist_to_sep_edge(com) < 0
