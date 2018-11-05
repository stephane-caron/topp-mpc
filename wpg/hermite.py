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

from numpy import dot

from TOPP.Trajectory import PiecewisePolynomialTrajectory


def interpolate_cubic_hermite(p0, p1, v0, v1):
    C3 = 2 * p0 - 2 * p1 + v0 + v1
    C2 = -3 * p0 + 3 * p1 - 2 * v0 - v1
    C1 = v0
    C0 = p0
    return C0, C1, C2, C3


def interpolate_ogh(p0, p1, v0, v1):
    """
    Interpolate an "Optimized Geometric Hermite" path between p0 and p1 with
    initial tangent parallel to v0 and final tangent parallel to v1.

    The output curve minimizes the integral of squared accelerations:

        $\\int_0^1 \\| \\ddot{B}(t) \\|^2 {\\rm d}{t}$

    Reference:
    <http://citeseerx.ist.psu.edu/viewdoc/download;jsessionid=73B65B5E213F7632A47E593533D23F61?doi=10.1.1.104.1622&rep=rep1&type=pdf>

    .. NOTE::

        Contrary to the referenced paper, we also impose that the output
        tangents share the sign of v0 and v1, respectively.
    """
    Delta = p1 - p0
    _Delta_v0 = dot(Delta, v0)
    _Delta_v1 = dot(Delta, v1)
    _v0_v0 = dot(v0, v0)
    _v0_v1 = dot(v0, v1)
    _v1_v1 = dot(v1, v1)
    a0 = (6 * _Delta_v0 * _v1_v1 - 3 * _Delta_v1 * _v0_v1) / (
        4 * _v0_v0 * _v1_v1 - _v0_v1 * _v0_v1)
    if a0 < 0:
        a0 *= -1
    a1 = (3 * _Delta_v0 * _v0_v1 - 6 * _Delta_v1 * _v0_v0) / (
        _v0_v1 * _v0_v1 - 4 * _v0_v0 * _v1_v1)
    if a1 < 0:
        a1 *= -1
    return interpolate_cubic_hermite(p0, p1, a0 * v0, a1 * v1)


def interpolate_houba(p0, p1, v0, v1, hack=False):
    """
    Interpolate a Hermite path between p0 and p1 with initial tangent parallel
    to v0 and final tangent parallel to v1. The output path B(s) minimizes
    (approximately) the uniform acceleration bound:

        minimize M s.t. \\forall s \\in [0, 1], \\|\\ddot{B}(s)\\|^s <= M

    See the paper for details.

    .. NOTE::

        We also impose that the output tangents share the sign of v0 and v1,
        respectively.
    """
    Delta = p1 - p0
    _Delta_v0 = dot(Delta, v0)
    _Delta_v1 = dot(Delta, v1)
    _v0_v0 = dot(v0, v0)
    _v0_v1 = dot(v0, v1)
    _v1_v1 = dot(v1, v1)
    b0 = 6 * (3 * _Delta_v0 * _v1_v1 - 2 * _Delta_v1 * _v0_v1) / (
        9 * _v0_v0 * _v1_v1 - 4 * _v0_v1 * _v0_v1)
    if b0 < 0:
        print "Hermite: b0 < 0"
        b0 *= -1
    b1 = 6 * (-2 * _Delta_v0 * _v0_v1 + 3 * _Delta_v1 * _v0_v0) / (
        9 * _v0_v0 * _v1_v1 - 4 * _v0_v1 * _v0_v1)
    if b1 < 0:
        print "Hermite: b1 < 0"
        b1 *= -1
    if hack:
        b1 *= 1.1
    return interpolate_cubic_hermite(p0, p1, b0 * v0, b1 * v1)


def interpolate_houba_topp(p0, p1, v0, v1, hack=False):
    """Wrapper to ``get_ubound_hermite_curve`` for use with TOPP."""
    C0, C1, C2, C3 = interpolate_houba(p0, p1, v0, v1, hack)
    path_str = "%f\n%d" % (1., 3)
    for k in xrange(3):
        path_str += "\n%f %f %f %f" % (C0[k], C1[k], C2[k], C3[k])
    return PiecewisePolynomialTrajectory.FromString(path_str)
