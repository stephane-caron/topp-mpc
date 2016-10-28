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

from pymanoid.misc import norm
from pymanoid.tasks import ContactTask, LinkPoseTask
from simulation import Process
from stance import Stance


class StateMachine(Process):

    transitions = {
        'DS-L': 'SS-L',
        'SS-L': 'DS-R',
        'DS-R': 'SS-R',
        'SS-R': 'DS-L'
    }

    def __init__(self, robot, contacts, com, free_foot, init_phase,
                 init_com_offset=None, cyclic=False, callback=None):
        """
        Create a new finite state machine.

        INPUT:

        - ``robot`` -- pymanoid robot model
        - ``contacts`` -- sequence of contacts
        - ``com`` -- PointMass object giving the current position of the COM
        - ``free_foot`` -- moving target for free foot motion
        - ``init_phase`` -- string giving the initial FSM state
        - ``init_com_offset`` -- used for initialization only
        - ``cyclic`` -- if True, contact sequence is looped over
        - ``callback`` -- (optional) function called after phase transitions

        .. NOTE::

            This function updates the position of ``com`` as a side effect.

        .. NOTE::

            Assumes that the motion starts at the end of a DS phase.
        """
        super(StateMachine, self).__init__()
        assert init_phase in ['DS-L', 'DS-R']  # kron
        first_stance = Stance(init_phase, contacts[0], contacts[1])
        if init_com_offset is not None:
            first_stance.com += init_com_offset
        com.set_pos(first_stance.com)
        self.callback = callback
        self.com = com
        self.contacts = contacts
        self.cur_phase = init_phase
        self.cur_stance = first_stance
        self.cyclic = cyclic
        self.free_foot = free_foot
        self.is_over = False
        self.nb_contacts = len(contacts)
        self.next_contact_id = 2 if init_phase == 'DS-R' else 3  # kroooon
        self.next_stance = None  # initialized below
        self.phase_id = -1  # 0 will be the first SS phase
        self.rem_time = 0.  # initial state is assumed at end of DS phase
        self.robot = robot
        self.thread = None
        self.thread_lock = None
        # pre-compute next stance (later done by step())
        self.next_stance = self.compute_next_stance()

    @property
    def next_contact(self):
        return self.contacts[self.next_contact_id]

    @property
    def next_phase(self):
        return self.transitions[self.cur_phase]

    def compute_next_stance(self):
        if self.next_phase == 'SS-L':
            left_foot = self.cur_stance.left_foot
            right_foot = None
        elif self.next_phase == 'DS-R':
            left_foot = self.cur_stance.left_foot
            right_foot = self.next_contact
        elif self.next_phase == 'SS-R':
            left_foot = None
            right_foot = self.cur_stance.right_foot
        elif self.next_phase == 'DS-L':
            left_foot = self.next_contact
            right_foot = self.cur_stance.right_foot
        else:  # should not happen
            assert False, "Unknown state: %s" % self.next_phase
        return Stance(self.next_phase, left_foot, right_foot)

    def on_tick(self, sim):
        """
        Update the FSM state after a tick of the control loop.

        INPUT:

        - ``sim`` -- instance of current simulation
        """
        if self.is_over:
            return
        if self.cur_stance.is_single_support:
            foot_target = self.next_stance.target_foot.effector_pose[4:]
            if norm(self.free_foot.p - foot_target) < 0.01:
                self.step()
        else:  # self.cur_stance.is_double_support
            # dist_to_edge = self.next_stance.dist_to_sep_edge(self.com.p)
            close_to_target = norm(self.cur_stance.com - self.com.p) < 0.05
            # small_velocity = norm(self.com.pd) < 0.1
            if close_to_target:
                self.com.set_velocity(0. * self.com.pd)  # kron
                self.step()

    def step(self):
        if self.next_contact_id == self.nb_contacts:
            self.is_over = True
            if self.callback is not None:
                self.callback()
            return
        next_stance = self.next_stance
        next_phase = self.next_phase
        if next_phase.startswith('DS'):
            self.next_contact_id += 1
            if self.next_contact_id >= self.nb_contacts and self.cyclic:
                self.next_contact_id -= self.nb_contacts
        else:  # next_phase.startswith('SS')
            next_pose = self.next_contact.pose
            if next_stance.left_foot is None:
                self.free_foot.reset(self.cur_stance.left_foot.pose, next_pose)
            else:  # next_stance.right_foot is None
                self.free_foot.reset(self.cur_stance.right_foot.pose, next_pose)
        self.cur_phase = next_phase
        self.cur_stance = next_stance
        self.next_stance = self.compute_next_stance()
        self.phase_id += 1
        self.update_robot_ik()
        if self.callback is not None:
            self.callback()

    def update_robot_ik(self):
        self.robot.ik.remove_task(self.robot.left_foot.name)
        self.robot.ik.remove_task(self.robot.right_foot.name)
        if self.cur_stance.left_foot is not None:
            left_foot_task = ContactTask(
                self.robot, self.robot.left_foot, self.cur_stance.left_foot)
        else:  # left_foot is free
            left_foot_task = LinkPoseTask(
                self.robot, self.robot.left_foot, self.free_foot)
        if self.cur_stance.right_foot is not None:
            right_foot_task = ContactTask(
                self.robot, self.robot.right_foot, self.cur_stance.right_foot)
        else:  # right_foot is free
            right_foot_task = LinkPoseTask(
                self.robot, self.robot.right_foot, self.free_foot)
        self.robot.ik.add_task(left_foot_task)
        self.robot.ik.add_task(right_foot_task)
