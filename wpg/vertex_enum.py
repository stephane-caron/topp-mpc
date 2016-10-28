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

from __future__ import division  # 1 / 2 == 0.5 rather than 0

from numpy import array, dot, hstack, zeros
from scipy.spatial import ConvexHull
from scipy.spatial.qhull import QhullError

from pymanoid import solve_lp


def enumerate_vertices_interior(B, c):
    """
    Compute the vertex representation of a polygon defined by:

        B * x <= c

    where x is a 2D vector. The origin [0, 0] should lie inside the polygon (c
    >= 0) in order to build the polar form. Call ``enumerate_vertices`` to
    compute the interior point automatically.

    INPUT:

    - ``B`` -- (2 x K) matrix
    - ``c`` -- vector of length K and positive coordinates

    OUTPUT:

    List of 2D vertices in counterclowise order.

    .. NOTE::

        Checking that (c > 0) is not optional. The rest of the algorithm can be
        executed when some coordinates c_i < 0, but the result would be wrong.
    """
    assert B.shape[1] == 2, "Input is not a polygon"
    assert all(c > 0), "Polygon should contain the origin"

    B_polar = hstack([
        (B[:, column] * 1. / c).reshape((B.shape[0], 1))
        for column in xrange(2)])

    def axis_intersection(i, j):
        ai, bi = c[i], B[i]
        aj, bj = c[j], B[j]
        cross = bi[0] * bj[1] - bj[0] * bi[1]
        # if ``cross`` is zero, your polygon may be unbounded
        x = (ai * bj[1] - aj * bi[1]) * 1. / cross
        y = (bi[0] * aj - bj[0] * ai) * 1. / cross
        return array([x, y])

    # QHULL OPTIONS:
    #
    # - ``Pp`` -- do not report precision problems
    # - ``Q0`` -- no merging with C-0 and Qx
    #
    # ``Q0`` avoids [this bug](https://github.com/scipy/scipy/issues/6484).
    # It slightly diminishes computation times (0.9 -> 0.8 ms on my machine)
    # but raises QhullError at the first sight of precision errors.
    #
    try:
        hull = ConvexHull([row for row in B_polar], qhull_options='Pp Q0')
    except QhullError:
        return None
    #
    # contrary to hull.simplices (which was not in practice), hull.vertices is
    # guaranteed to be in counterclockwise order for 2-D (see scipy doc)
    #
    simplices = [(hull.vertices[i], hull.vertices[i + 1])
                 for i in xrange(len(hull.vertices) - 1)]
    simplices.append((hull.vertices[-1], hull.vertices[0]))
    vertices = [axis_intersection(i, j) for (i, j) in simplices]
    return vertices


def enumerate_vertices(B, c):
    """
    Compute the vertex representation of a polygon defined by:

        B * x <= c

    where x is a 2D vector. Computes an interior point before calling
    ``enumerate_vertices_interior``.

    INPUT:

    - ``B`` -- (2 x K) matrix
    - ``c`` -- vector of length K and positive coordinates

    OUTPUT:

    List of 2D vertices in counterclowise order.
    """
    if all(c > 0):
        return enumerate_vertices_interior(B, c)
    nb_trials = 0
    x_sum = zeros(2)
    for d in [array([+1., 0.]), array([-1., 0.]), array([0., -1.])]:
        x_new = solve_lp(d, B, c)
        if x_new is None:
            continue
        x_sum += x_new
        nb_trials += 1
        if nb_trials < 2:
            continue
        x_bary = x_sum / nb_trials
        cc = c - dot(B, x_bary)
        if all(cc > 0):  # x_bary is inside the polygon
            hull = enumerate_vertices_interior(B, cc)
            if hull is None:
                return None
            return [y + x_bary for y in hull]
    return None
