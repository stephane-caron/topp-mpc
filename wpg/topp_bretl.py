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

from cvxopt import matrix as cvxopt_matrix
from cvxopt.solvers import lp as solve_lp
from numpy import array, cos, cross, dot, hstack, pi, sin, vstack, zeros
from numpy import linalg

from pymanoid import get_gravity


"""
Polygon projection
==================
"""


def OptimizeDirection(vdir, lp):
    """Optimize in one direction."""
    lp_G, lp_h = lp
    lp_q = cvxopt_matrix(-vdir)
    sol = solve_lp(lp_q, lp_G, lp_h, solver='glpk')
    z = sol['x']
    if z is not None:
        z = array(z).reshape((lp_q.size[0],))
        return True, z
    else:
        print "Failed"
        return False, 0


def __compute_topp_polygon_bretl(cwc, p, ps, pss, sdd_max=None, sd_max=None):
    """Compute the polygon in the (sddot,sdot^2) plane using Hauser method."""
    g = get_gravity()
    b1 = dot(cwc, hstack([ps, cross(p, ps)]))
    b2 = dot(cwc, hstack([pss, cross(p, pss)]))
    b3 = dot(cwc, hstack([g, cross(p, g)]))
    n = len(b1)
    G = zeros((n+1, 2))
    G[0, :] = [0, -1]
    G[1:, 0] = b1
    G[1:, 1] = b2
    h = zeros(n+1)
    h[1:] = b3

    if sdd_max is not None:
        G = vstack([array([[1, 0], [-1, 0]]),  G])
        h = hstack([array([sdd_max, sdd_max]), h])
    if sd_max is not None:
        sd_max_2 = sd_max ** 2
        G = vstack([array([[0, 1], [0, -1]]),  G])
        h = hstack([array([sd_max_2, sd_max_2]), h])

    lp = cvxopt_matrix(G), cvxopt_matrix(h)
    res, z1 = OptimizeDirection(array([1., 0.]), lp)
    if not res:
        return None
    res, z2 = OptimizeDirection(array([cos(2*pi/3), sin(2*pi/3)]), lp)
    if not res:
        return None
    res, z3 = OptimizeDirection(array([cos(4*pi/3), sin(4*pi/3)]), lp)
    if not res:
        return None
    v1 = Vertex(z1)
    v2 = Vertex(z2)
    v3 = Vertex(z3)
    P = Polygon()
    P.fromVertices(v1, v2, v3)
    P.iter_expand(lp, 100)
    return P


def compute_topp_polygon_bretl(cwc, p, ps, pss, sdd_max=None, sd_max=None):
    P = __compute_topp_polygon_bretl(cwc, p, ps, pss, sdd_max, sd_max)
    if P is None:
        return None
    P.sort_vertices()
    vertices_list = P.export_vertices()
    vertices_list.reverse()
    return [(v.x, v.y) for v in vertices_list]


"""
Polygon classes
===============
"""


class Vertex:

    def __init__(self, z):
        self.x = z[0]
        self.y = z[1]
        self.next = None
        self.expanded = False

    def length(self):
        return linalg.norm([self.x - self.next.x, self.y - self.next.y])

    def expand(self, lp):
        v1 = self
        v2 = self.next
        v = array([v2.y - v1.y, v1.x - v2.x])  # orthogonal direction to edge
        v = 1/linalg.norm(v) * v
        res, z = OptimizeDirection(v, lp)
        if not res:
            self.expanded = True
            return False, None
        xopt, yopt = z
        if(abs(cross([xopt-v1.x, yopt-v1.y], [v1.x-v2.x, v1.y-v2.y])) < 1e-2):
            self.expanded = True
            return False, None
        else:
            vnew = Vertex([xopt, yopt])
            vnew.next = self.next
            self.next = vnew
            self.expanded = False
            return True, vnew

    def Plot(self):
        import pylab
        pylab.plot([self.x, self.next.x], [self.y, self.next.y])

    def Print(self):
        print self.x, self.y, "to", self.next.x,  self.next.y


class Polygon:

    def __init__(self):
        pass

    def fromVertices(self, v1, v2, v3):
        v1.next = v2
        v2.next = v3
        v3.next = v1
        self.vertices = [v1, v2, v3]

    def all_expanded(self):
        for v in self.vertices:
            if not v.expanded:
                return False
        return True

    def iter_expand(self, qpconstraints, maxiter=10):
        """
        Returns True if there's a edge that can be expanded, and expands that
        edge, otherwise returns False.
        """
        niter = 0
        v = self.vertices[0]
        while not self.all_expanded() and niter < maxiter:
            if not v.expanded:
                res, vnew = v.expand(qpconstraints)
                if res:
                    self.vertices.append(vnew)
                    niter += 1
            else:
                v = v.next

    def sort_vertices(self):
        """
        Export the vertices starting from the left-most and going clockwise.

        Assumes every vertices are on the positive halfplane.
        """
        minsd = 1e10
        ibottom = 0
        for i in range(len(self.vertices)):
            v = self.vertices[i]
            if (v.y + v.next.y) < minsd:
                ibottom = i
                minsd = v.y + v.next.y
        for v in self.vertices:
            v.checked = False
        vcur = self.vertices[ibottom]
        newvertices = []
        while not vcur.checked:
            vcur.checked = True
            newvertices.append(vcur)
            vcur = vcur.next
        newvertices.reverse()
        vfirst = newvertices.pop(-1)
        newvertices.insert(0, vfirst)
        self.vertices = newvertices

    def export_vertices(self, threshold=1e-2):
        export_list = [self.vertices[0]]
        for i in range(1, len(self.vertices)-1):
            vcur = self.vertices[i]
            vlast = export_list[-1]
            if(linalg.norm([vcur.x-vlast.x, vcur.y-vlast.y])) > threshold:
                export_list.append(vcur)
        export_list.append(self.vertices[-1])  # always add last vertex
        return export_list

    def Plot(self):
        import pylab
        pylab.hold("on")
        for v in self.vertices:
            v.Plot()

    def Print(self):
        print "Polygon contains vertices"
        for v in self.vertices:
            v.Print()
