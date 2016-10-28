#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# Copyright (C) 2016 Stephane Caron <stephane.caron@normalesup.org>
#
# This file is part of 3d-mpc <https://github.com/stephane-caron/3d-mpc>.
#
# 3d-mpc is free software: you can redistribute it and/or modify it under
# the terms of the GNU General Public License as published by the Free Software
# Foundation, either version 3 of the License, or (at your option) any later
# version.
#
# 3d-mpc is distributed in the hope that it will be useful, but WITHOUT
# ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
# FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more
# details.
#
# You should have received a copy of the GNU General Public License along with
# 3d-mpc. If not, see <http://www.gnu.org/licenses/>.

from numpy import sqrt


class AvgStdEstimator(object):

    def __init__(self):
        self.x = 0.
        self.x2 = 0.
        self.n = 0

    def add(self, v):
        self.x += v
        self.x2 += v ** 2
        self.n += 1

    def get_all(self):
        if self.n < 1:
            return (None, None, 0)
        avg = self.x / self.n
        if self.n == 1:
            return (avg, 0., 1)
        unbiased = sqrt(self.n * 1. / (self.n - 1))
        std = unbiased * sqrt(self.x2 / self.n - avg ** 2)
        return (avg, std, self.n)
