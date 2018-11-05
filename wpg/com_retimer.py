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

import topp_param

from TOPP.TOPPbindings import TOPPInstance
from hermite import interpolate_houba_topp
from pymanoid.misc import norm, normalize
from retimer import BodyRetimer


class COMRetimer(BodyRetimer):

    def __init__(self, com, fsm):
        super(COMRetimer, self).__init__(com)
        self.debug_polygons = False
        self.fsm = fsm
        self.switch_time = None

    def update(self, switch_time=None):
        self.switch_time = switch_time
        return super(COMRetimer, self).update()

    def interpolate_path(self):
        target_p = self.fsm.next_stance.com
        target_pd = self.fsm.next_stance.comd
        p0 = self.body.p
        p1 = target_p
        if norm(p1 - p0) < 1e-3:
            return None, None
        v1 = target_pd
        if norm(self.body.pd) > 1e-4:
            v0 = self.body.pd
            path = interpolate_houba_topp(p0, p1, v0, v1, hack=True)
            sd_beg = norm(v0) / norm(path.Evald(0.))
        else:  # choose initial direction
            delta = normalize(p1 - p0)
            ref = normalize(self.fsm.cur_stance.comd)
            v0 = 0.7 * delta + 0.3 * ref
            path = interpolate_houba_topp(p0, p1, v0, v1, hack=True)
            sd_beg = 0.
        return path, sd_beg

    def find_switch_index(self, path, i_beg, i_end):
        while i_end > i_beg + 1:
            # Let:
            # p_beg = path.Eval(i_beg * self.discrtimestep)
            # p_end = path.Eval(i_end * self.discrtimestep)
            # Loop invariant: p_beg in SEP, p_end outside
            i_mid = (i_beg + i_end) / 2
            p_mid = path.Eval(i_mid * self.discrtimestep)
            if self.fsm.cur_stance.is_outside_sep(p_mid):
                i_end = i_mid
            else:  # p_mid is inside the SEP
                i_beg = i_mid
        return i_beg

    def compute_sdd_max(self, s_switch, sd_beg):
        if self.switch_time is None:  # double support
            return None
        sd_switch = s_switch / self.switch_time
        return (sd_switch - sd_beg) * (sd_switch + sd_beg) / (2 * s_switch)

    def compute_topp_instance(self, path, sd_beg):
        id_traj = "1.0\n1\n0.0 1.0"
        discrtimestep = self.discrtimestep
        ndiscrsteps = int((path.duration + 1e-10) / discrtimestep) + 1
        sdd_max_ds = 10.  # large value for double-support
        sd_max = 10.

        if self.fsm.cur_stance.is_single_support:
            i_switch = self.find_switch_index(path, 0, ndiscrsteps - 1)
            if i_switch < 1:
                # s_switch is below discrtimestep, keep current preview
                return None
            s_switch = i_switch * discrtimestep
            sdd_max_before_switch = self.compute_sdd_max(s_switch, sd_beg)
            if sdd_max_before_switch < 0.:
                # print "Hu ohhhh..."
                # self.debug_polygons = True
                # raw_input()
                return None
        else:  # cur_stance is double-support
            s_switch = 42.  # whatever > 1
            sdd_max_before_switch = sdd_max_ds
        self.s_switch = s_switch  # for plots

        constraints = str(discrtimestep)
        constraints += "\n0.0"  # no velocity limits
        constraints += "\n"
        for i in range(ndiscrsteps):
            s = i * discrtimestep
            p = path.Eval(s)
            ps = path.Evald(s)
            pss = path.Evaldd(s)
            if s <= s_switch:
                cwc = self.fsm.cur_stance.cwc_topp
                sdd_max = sdd_max_before_switch
            else:  # s >= s_switch
                cwc = self.fsm.next_stance.cwc_topp
                sdd_max = sdd_max_ds
            # nps = norm(ps)
            # sd_max = min(10., max_com_vel / nps) if nps > 1e-3 else 10.
            hull = topp_param.check_compute_polygon_hull(
                cwc, p, ps, pss, sdd_max=sdd_max, sd_max=sd_max)
            if hull is None:
                raise Exception("Cannot evaluate constraints at s=%.2f" % s)
            constraints += topp_param.hull_to_constraints(hull)
            if i < ndiscrsteps - 1:
                constraints += "\n"
            if self.debug_polygons and i < 1:  # debug polygons
                import pylab
                from pymanoid.misc import plot_polygon
                pylab.ion()
                pylab.clf()
                plot_polygon(hull)
                pylab.title("$s$ = %f" % s)
                pylab.xlabel('$\\ddot{s}$')
                pylab.ylabel('$\\dot{s}^2$')
                pylab.grid(True)
                if sdd_max is not None:
                    pylab.axis((-1.5 * sdd_max, 1.5 * sdd_max, 0, 10))
                raw_input()

        inst = TOPPInstance(None, "PolygonConstraints", constraints, id_traj)
        inst.integrationtimestep = 1e-3
        return inst

    def plot_profiles(self):
        import pylab
        super(COMRetimer, self).plot_profiles()
        if self.s_switch < 1.:
            pylab.plot(
                [self.s_switch, self.s_switch], [0, 100], "k--", linewidth=2)
