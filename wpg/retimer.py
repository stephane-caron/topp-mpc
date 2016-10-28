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

from TOPP import TOPPpy
from TOPP.Errors import TOPP_OK
from TOPP.Trajectory import PiecewisePolynomialTrajectory
from TOPP.Utilities import Interpolate3rdDegree


def interpolate_path_cuong(q_beg, q_end, qs_beg, qs_end):
    """Interpolate a 3rd-degree path."""
    path_str = ''
    dim = len(q_beg)
    duration = 1.  # all path durations are normalized to 1.
    path_str += "%f\n%d" % (duration, dim)
    for k in range(dim):
        a, b, c, d = Interpolate3rdDegree(
            q_beg[k], q_end[k], qs_beg[k], qs_end[k], duration)
        path_str += "\n%f %f %f %f" % (d, c, b, a)
    return PiecewisePolynomialTrajectory.FromString(path_str)


class PathRetimingWrapper(object):

    def __init__(self):
        self.path = None
        self.s_traj = None

    def update(self, path, s_traj):
        self.path = path
        self.s_traj = s_traj

    @property
    def duration(self):
        return self.s_traj.duration

    def Eval(self, t):
        s = self.s_traj.Eval(t)
        return self.path.Eval(s)

    def Evald(self, t):
        s = self.s_traj.Eval(t)
        sd = self.s_traj.Evald(t)
        return self.path.Evald(s) * sd

    def Evaldd(self, t):
        s = self.s_traj.Eval(t)
        sd = self.s_traj.Evald(t)
        sdd = self.s_traj.Evaldd(t)
        return self.path.Evald(s) * sdd + self.path.Evaldd(s) * sd * sd


class BodyRetimer(object):

    def __init__(self, body):
        self.body = body
        self.discrtimestep = 0.1
        self.output_wrapper = PathRetimingWrapper()
        self.reparamstep = 0.01  # defaults to self.discrtimestep

    def update(self):
        path, sd_beg = self.interpolate_path()
        if path is None:
            return False

        try:
            new_topp = self.compute_topp_instance(path, sd_beg)
        except Exception as e:
            print "%s retimer update:" % type(self).__name__, e
            return False

        if new_topp is None:
            return False
        self.topp = new_topp

        sd_end = 0.0
        return_code = self.topp.RunComputeProfiles(sd_beg, sd_end)
        if return_code != TOPP_OK:
            # from TOPP.Errors import MESSAGES as TOPP_MESSAGES
            # print "TOPP error: %s" % TOPP_MESSAGES[return_code]
            return False

        self.topp.ReparameterizeTrajectory(self.reparamstep)
        self.topp.WriteResultTrajectory()
        s_traj = PiecewisePolynomialTrajectory.FromString(
            self.topp.restrajectorystring)
        if abs(self.topp.resduration - s_traj.duration) > 0.1:
            print ""
            print "--"
            print "Trajectory duration:", self.topp.resduration
            print "check:", s_traj.duration
            print ""
        sdd_beg = s_traj.Evaldd(0.)[0]
        alpha = self.topp.GetAlpha(0., sd_beg)
        beta = self.topp.GetBeta(0., sd_beg)
        sdd_beg = min(beta, max(alpha, sdd_beg))  # may be outside
        self.sd_beg = sd_beg
        self.sdd_beg = sdd_beg
        comd_beg = path.Evald(0) * sd_beg
        comdd_beg = path.Evald(0) * sdd_beg + path.Evaldd(0) * sd_beg * sd_beg
        self.output_wrapper.update(path, s_traj)
        self.output_wrapper.comd_beg = comd_beg
        self.output_wrapper.comdd_beg = comdd_beg
        return True

    def get_trajectory(self):
        return self.output_wrapper

    def plot_profiles(self):
        import pylab
        self.topp.WriteProfilesList()
        self.topp.WriteSwitchPointsList()
        profileslist = TOPPpy.ProfilesFromString(
            self.topp.resprofilesliststring)
        switchpointslist = TOPPpy.SwitchPointsFromString(
            self.topp.switchpointsliststring)
        TOPPpy.PlotProfiles(profileslist, switchpointslist)
        # TOPPpy.PlotAlphaBeta(topp)
        pylab.title("%s phase profile" % type(self).__name__)
        pylab.axis([0, 1, 0, 10])

    def check_sdd_beg(self):
        alpha = self.topp.GetAlpha(0, self.sd_beg)
        beta = self.topp.GetBeta(0, self.sd_beg)
        sdd_beg = self.output_wrapper.s_traj.Evaldd(0)[0]
        print "alpha =", alpha
        print "sdd_beg =", sdd_beg
        print "beta =", beta
