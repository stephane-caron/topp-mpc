#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# Copyright (C) 2016 Quang-Cuong Pham <cuong.pham@normalesup.org>
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

from numpy import array, cross, dot, hstack, transpose, vstack

from pymanoid import get_gravity
from topp_bretl import compute_topp_polygon_bretl
from vertex_enum import enumerate_vertices


def compute_inequalities(cwc, p, ps, pss):
    """
    Compute the constraints (B, c) such that B * [sdd sd^2] <= c.

    INPUT:

    - ``cwc`` -- (K x 6) matrix
    - ``p`` -- vector of length 6
    - ``ps`` -- vector of length 6
    - ``pss`` -- vector of length 6

    OUTPUT:

    - ``B`` -- ((K+1) x 6) matrix
    - ``c`` -- vector of length K+1
    """
    g = get_gravity()
    b1 = dot(cwc, hstack([ps, cross(p, ps)]))
    b2 = dot(cwc, hstack([pss, cross(p, pss)]))
    c = dot(cwc, hstack([g, cross(p, g)]))
    B = transpose(vstack([b1, b2]))
    return \
        vstack([array([0., -1.]), B]), \
        hstack([array(0.), c])


def check_compute_polygon_hull(cwc, p, ps, pss, sdd_max=None, sd_max=None):
    """
    Compute the vertex representation of a polygon defined by:

        B * x <= c

    where x is a 2D vector. If the origin (0,0) is not in the interior of the
    polygon then find one interior point and shift the origin. Then call
    Stephane's compute_polygon_hull.

    If everything fails, then call Bretl method.

    INPUT:

    - ``B`` -- (2 x K) matrix
    - ``c`` -- vector of length K and positive coordinates

    OUTPUT:

    List of 2D vertices in counter-clockwise order.
    """
    B, c = compute_inequalities(cwc, p, ps, pss)
    if sdd_max is not None:
        B = vstack([array([[1., 0.], [-1., 0.]]),  B])
        c = hstack([array([sdd_max, sdd_max]), c])
    if sd_max is not None:
        sd_max_2 = sd_max ** 2
        B = vstack([array([[0, 1], [0, -1]]),  B])
        c = hstack([array([sd_max_2, sd_max_2]), c])
    hull = enumerate_vertices(B, c)
    if hull is None:
        print "\nCannot find a strict interior point, switching to Bretl\n"
        return compute_topp_polygon_bretl(cwc, p, ps, pss, sdd_max, sd_max)
    return hull


def hull_to_constraints(hull):
    """
    Convert the hull given by ``compute_polygon_hull`` with vertices ordered
    counter-clockwise into the constraint string of TOPP (where vertices are
    ordered in clockwise order).
    """
    # Find the lowest vertex
    lowest_vertex = hull[0]
    lowest_index = 0
    for i in range(len(hull)):
        if hull[i][1] < lowest_vertex[1]:
            lowest_vertex = hull[i]
            lowest_index = i
    i = lowest_index
    while True:
        i -= 1
        if hull[i][1] > lowest_vertex[1] + 1e-5:
            break
    i += 1
    # Create the constraint string
    constraintstring = ""
    for j in range(len(hull)):
        constraintstring += str(hull[i][0]) + " " + str(abs(hull[i][1]))
        #  Note that it's sd^2 which is returned
        if j < len(hull)-1:
            constraintstring += " "
        i -= 1
    return constraintstring
