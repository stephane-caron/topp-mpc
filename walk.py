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

import IPython
import os
import pylab
import re
import sys
import time

from numpy import arange, hstack, pi, sin, zeros
from numpy.random import random, seed

script_path = os.path.realpath(__file__)
sys.path = [os.path.dirname(script_path) + '/pymanoid'] + sys.path

import pymanoid

from pymanoid import Contact, PointMass
from pymanoid import draw_force, draw_line, draw_point, draw_polygon
from pymanoid.tasks import COMTask, ContactTask, DOFTask
from pymanoid.tasks import MinCAMTask
from pymanoid.tasks import MinVelocityTask
from wpg.buffer import PreviewBuffer
from wpg.free_foot import FreeFoot
from wpg.fsm import StateMachine
from wpg.simulation import Process, Simulation
from wpg.topp_control import TOPPPreviewControl

if os.path.isfile('HRP4R.dae'):
    from pymanoid.robots import HRP4 as RobotModel
else:
    from pymanoid.robots import JVRC1 as RobotModel


def generate_contacts():
    FOOT_X, FOOT_Y = 0.2, 0.1
    dstep = 0.7  # [m]
    friction = 0.9
    height = 1.4  # [m]
    leg_spread = 0.4  # [m]
    nb_steps = 40
    nb_waves = 2
    rpy_roughness = 0.5  # [rad]
    length = (nb_steps / 2) * dstep
    steps = []
    for x in arange(0., length, dstep):
        theta = 2 * pi * nb_waves * x / length
        left_foot = Contact(
            X=FOOT_X,
            Y=FOOT_Y,
            pos=[x,
                 leg_spread * (0.1 * sin(theta) + 0.5),
                 .5 * height * sin(theta)],
            rpy=rpy_roughness * (random(3) - 0.5),
            friction=friction,
            visible=True)
        left_foot.set_pitch(sin(theta - pi / 2) / 2)
        right_foot = Contact(
            X=FOOT_X,
            Y=FOOT_Y,
            pos=[x + 0.5 * dstep,
                 leg_spread * (0.1 * sin(theta) - 0.5),
                 .5 * height * sin(theta + .5 * 0.5)],
            rpy=rpy_roughness * (random(3) - 0.5),
            friction=friction,
            visible=True)
        right_foot.set_pitch(left_foot.pitch)
        steps.append(left_foot)
        steps.append(right_foot)
    return steps


def dash_graph_handles(handles):
    for i in xrange(len(handles)):
        if i % 2 == 0:
            handles[i] = None


class DrawerProcess(Process):

    def __init__(self):
        super(DrawerProcess, self).__init__()
        self.handles = []

    def hide(self):
        for handle in self.handles:
            if handle:
                handle.SetShow(False)

    def show(self):
        for handle in self.handles:
            if handle:
                handle.SetShow(True)


class ForceChecker(DrawerProcess):

    def __init__(self, force_scale=0.002):
        super(ForceChecker, self).__init__()
        self.comdd = zeros(3)
        self.force_scale = force_scale
        self.last_bkgnd_switch = None

    def on_tick(self, sim):
        """Find supporting contact forces at each COM acceleration update."""
        comdd = 0.9 * preview_buffer.comdd + 0.1 * self.comdd
        gravity = pymanoid.get_gravity()
        wrench = hstack([robot.mass * (comdd - gravity), zeros(3)])
        support = fsm.cur_stance.find_supporting_forces(
            wrench, preview_buffer.com.p, robot.mass)
        if not support:
            # self.handles = []
            # viewer.SetBkgndColor([.8, .4, .4])
            # self.last_bkgnd_switch = time.time()
            pass
        else:
            self.handles = [
                draw_force(c, fc, scale=self.force_scale)
                for (c, fc) in support]
        if self.last_bkgnd_switch is not None \
                and time.time() - self.last_bkgnd_switch > 0.2:
            # let's keep epilepsy at bay
            viewer.SetBkgndColor([.6, .6, .8])
            self.last_bkgnd_switch = None
        self.comdd = comdd
        self.comd_handle = draw_force(com.p, com.pd, scale=0.5)

    def hide(self):
        super(ForceChecker, self).hide()
        if self.comd_handle:
            self.comd_handle.SetShow(False)

    def show(self):
        super(ForceChecker, self).show()
        if self.comd_handle:
            self.comd_handle.SetShow(True)


class PreviewDrawer(DrawerProcess):

    def on_tick(self, sim):
        self.handles = []
        com_path = mpc.com_retimer.output_wrapper.path
        if com_path is not None:
            ds = mpc.com_retimer.discrtimestep
            self.draw_path(com_path, 'r', ds)
        foot_path = mpc.foot_retimer.output_wrapper.path
        if foot_path is not None:
            ds = mpc.foot_retimer.discrtimestep
            self.draw_path(foot_path, 'b', ds)

    def draw_path(self, path, color, ds):
        self.handles.append(
            draw_point(path.Eval(0), color='m', pointsize=0.007))
        for s in arange(ds, path.duration + ds, ds):
            self.handles.append(
                draw_point(path.Eval(s), color=color, pointsize=0.01))
            self.handles.append(
                draw_line(path.Eval(s - ds), path.Eval(s), color=color,
                          linewidth=3))


class WindowRecorder(Process):

    def __init__(self, record_video):
        super(WindowRecorder, self).__init__()
        print "Please click on the OpenRAVE window."
        line = os.popen('/usr/bin/xwininfo | grep "Window id:"').readlines()[0]
        self.frame_index = 0
        self.record_video = record_video
        self.window_id = "0x%s" % re.search('0x([0-9a-f]+)', line).group(1)

    def on_tick(self, sim):
        if self.record_video:
            fname = './recording/camera/%05d.png' % (self.frame_index)
            self.grab(fname)
            self.frame_index += 1

    def grab(self, fname):
        os.system('import -window %s %s' % (self.window_id, fname))


class SEPDrawer(Process):

    def __init__(self):
        super(SEPDrawer, self).__init__()
        self.ss_handles = None
        self.ds_handles = None

    def on_tick(self, sim):
        if fsm.next_stance is None:
            return
        if fsm.cur_stance.is_single_support:
            ss_stance = fsm.cur_stance
            ds_stance = fsm.next_stance
        else:  # fsm.cur_stance.is_double_support:
            ss_stance = fsm.next_stance
            ds_stance = fsm.cur_stance
        sep_height = 3.  # [m]
        self.ss_handles = draw_polygon(
            [(x[0], x[1], sep_height) for x in ss_stance.sep],
            normal=[0, 0, 1], color='c')
        self.ds_handles = draw_polygon(
            [(x[0], x[1], sep_height) for x in ds_stance.sep],
            normal=[0, 0, 1], color='y')


class TrajectoryDrawer(DrawerProcess):

    def __init__(self, body, combined='b-', color=None, linewidth=3,
                 linestyle=None):
        super(TrajectoryDrawer, self).__init__()
        color = color if color is not None else combined[0]
        linestyle = linestyle if linestyle is not None else combined[1]
        assert linestyle in ['-', '.']
        self.body = body
        self.color = color
        self.last_pos = body.p
        self.linestyle = linestyle
        self.linewidth = linewidth
        self.parity = True

    def on_tick(self, sim):
        if self.linestyle == '-' or self.parity:
            self.handles.append(draw_line(
                self.last_pos, self.body.p, color=self.color,
                linewidth=self.linewidth))
        self.last_pos = self.body.p
        self.parity = not self.parity


class IKProcess(Process):

    def __init__(self, robot):
        super(IKProcess, self).__init__()
        self.last_cost = 100000.
        self.robot = robot

    def on_tick(self, sim):
        self.robot.step_ik(sim.dt)
        if fsm.is_over:
            cost = self.robot.ik.compute_cost(sim.dt)
            if abs(cost - self.last_cost) / self.last_cost < 1e-3:
                robot_ik.stop()
            self.last_cost = cost


class ProfileDrawer(Process):

    def on_tick(self, sim):
        pylab.clf()
        mpc.plot_profiles()


class CameraTravelling(Process):

    def __init__(self, viewer, dist):
        super(CameraTravelling, self).__init__()
        self.dist = dist
        self.camera_transform = None
        self.viewer = viewer

    def on_tick(self, sim):
        if self.camera_transform is None:  # initialize when simulation starts
            self.camera_transform = self.viewer.GetCameraTransform()
        self.camera_transform[0, 3] = com.x + self.dist
        self.viewer.SetCamera(self.camera_transform)


class PostureRecorder(Process):

    def __init__(self, robot, period):
        super(PostureRecorder, self).__init__()
        self.period = period
        self.postures = []
        self.robot = robot

    def on_tick(self, sim):
        if sim.tick_time % self.period == 0:
            self.postures.append(self.robot.q)

    def make_figure(self, window_recorder, indices=None):
        if sim.is_running:
            sim.stop()
        if indices is None:
            indices = xrange(len(self.postures))
        fnames = []
        for i in indices:
            self.robot.set_dof_values(self.postures[i])
            time.sleep(0.5)
            fname = 'recording/postures/%02d.png' % i
            window_recorder.grab(fname)
            os.system('mogrify -channel rgba -alpha set -fill none '
                      '-draw \'color 100,100 replace\' %s' % fname)
            fnames.append(fname)
            if i == indices[0]:
                com_traj_drawer.hide()
                left_foot_traj_drawer.hide()
                preview_drawer.hide()
                right_foot_traj_drawer.hide()
                hide_contacts()
        os.system('convert %s -layers merge postures.png' % ' '.join(fnames))
        os.system('eog postures.png')
        com_traj_drawer.show()
        left_foot_traj_drawer.show()
        preview_drawer.show()
        right_foot_traj_drawer.show()
        show_contacts()


def make_figure():
    free_foot.set_visible(False)
    viewer.SetBkgndColor([1, 1, 1])
    robot.suntan()
    robot.set_transparency(0.1)
    force_checker.hide()
    target_box.set_visible(False)
    # indices = [0, 50, 94, 165, 208, 238, 267, 300, 350]  # used with period=5
    # posture_recorder.make_figure(window_recorder, indices)


def hide_contacts():
    for contact in fsm.contacts:
        contact.hide()


def show_contacts():
    for contact in fsm.contacts:
        contact.show()


from wpg.stats import AvgStdEstimator
ds_times = AvgStdEstimator()
ss_times = AvgStdEstimator()
last_time = 0.


def fsm_step_callback():
    global last_time
    rtime = sim.tick_time * sim.dt
    if fsm.cur_stance.is_double_support:
        ds_times.add(rtime - last_time)
        print "Finished DS in", rtime - last_time
    else:  # fsm.cur_stance.is_single_support
        ss_times.add(rtime - last_time)
        print "Finished SS in", rtime - last_time
    last_time = rtime
    print "DS:", ds_times.get_all()
    print "SS:", ss_times.get_all()
    if fsm.is_over:
        mpc.stop()
        preview_buffer.stop()
    elif fsm.next_stance is not None:
        target_box.set_pos(fsm.next_stance.com)


def initialize_robot():
    robot.set_dof_values(robot.q_halfsit)
    active_dofs = robot.chest + robot.free
    active_dofs += robot.left_leg + robot.right_leg
    robot.set_active_dofs(active_dofs)
    robot.init_ik(
        gains={
            'com': 1.,
            'contact': 1.,
            'link_pose': 1.,
        },
        weights={
            'com': 10.,
            'contact': 1000.,
            'link_pose': 100.,
        })
    robot.set_dof_values([1.], [robot.TRANS_Z])  # warm-start PG

    stance = fsm.cur_stance
    com_task = COMTask(robot, stance.com)
    robot.ik.add_task(com_task)
    robot.ik.add_task(ContactTask(robot, robot.left_foot, stance.left_foot))
    robot.ik.add_task(ContactTask(robot, robot.right_foot, stance.right_foot))
    robot.ik.add_task(MinVelocityTask(robot, weight=1.))
    robot.solve_ik(max_it=50)

    # active_dofs += [robot.L_ELBOW_P, robot.R_ELBOW_P]
    active_dofs += [robot.L_SHOULDER_R, robot.R_SHOULDER_R]
    robot.set_dof_limits([+0.15], [+1.], [robot.L_SHOULDER_R])
    robot.set_dof_limits([-1.], [-0.15], [robot.R_SHOULDER_R])
    robot.set_active_dofs(active_dofs)
    com_task.update_target(preview_buffer.com)
    robot.ik.add_task(
        MinCAMTask(robot, weight=1.))
    robot.ik.add_task(
        DOFTask(robot, robot.CHEST_P, 0.2, gain=0.9, weight=0.05))
    robot.ik.add_task(
        DOFTask(robot, robot.CHEST_Y, 0., gain=0.9, weight=0.05))
    robot.ik.add_task(
        DOFTask(robot, robot.ROT_P, 0., gain=0.9, weight=0.05))
    robot.solve_ik()


def record_video():
    window_recorder.record_video = True
    camera_travel.on_tick(sim)
    time.sleep(2)
    sim.start()


if __name__ == "__main__":
    seed(24)
    pylab.ion()
    pymanoid.init(set_viewer=False)
    robot = RobotModel(download_if_needed=True)
    robot.set_transparency(0.3)

    env = pymanoid.get_env()
    env.SetViewer('qtcoin')
    pymanoid.env.set_default_background_color()
    viewer = pymanoid.get_viewer()
    viewer.SetCamera([
        [5.39066316e-01,   3.61154816e-01,  -7.60903874e-01, 6.57677031e+00],
        [8.42254221e-01,  -2.26944015e-01,   4.88982864e-01, -2.9774201e+00],
        [3.91593606e-03,  -9.04468691e-01,  -4.26522042e-01, 2.25269456e+00],
        [0.00000000e+00,   0.00000000e+00,   0.00000000e+00, 1.00000000e+00]])

    contacts = generate_contacts()
    com = PointMass([0, 0, 0], robot.mass, visible=False)
    free_foot = FreeFoot(color='c', visible=False)
    preview_buffer = PreviewBuffer(com, free_foot)
    fsm = StateMachine(
        robot, contacts, com, free_foot, 'DS-R', callback=fsm_step_callback)
    target_box = PointMass(fsm.cur_stance.com, 30., color='g')
    mpc = TOPPPreviewControl(com, free_foot, fsm, preview_buffer)
    initialize_robot()

    camera_travel = CameraTravelling(viewer, dist=4)
    com_traj_drawer = TrajectoryDrawer(com, 'b-')
    force_checker = ForceChecker()
    left_foot_traj_drawer = TrajectoryDrawer(robot.left_foot, 'g-')
    posture_recorder = PostureRecorder(robot, period=5)
    preview_drawer = PreviewDrawer()
    profile_plotter = ProfileDrawer()
    right_foot_traj_drawer = TrajectoryDrawer(robot.right_foot, 'r-')
    robot_ik = IKProcess(robot)
    window_recorder = None
    # window_recorder = WindowRecorder(record_video=False)
    sep_drawer = SEPDrawer()

    sim = Simulation(dt=3e-2)
    sim.schedule(fsm)
    sim.schedule(mpc)
    sim.schedule(preview_buffer)
    sim.schedule(robot_ik)

    # sim.schedule_extra(camera_travel)
    sim.schedule_extra(com_traj_drawer)
    sim.schedule_extra(force_checker)
    sim.schedule_extra(left_foot_traj_drawer)
    sim.schedule_extra(posture_recorder)
    sim.schedule_extra(preview_drawer)
    # sim.schedule_extra(profile_plotter)
    sim.schedule_extra(right_foot_traj_drawer)
    # sim.schedule_extra(window_recorder)
    # sim.schedule_extra(sep_drawer)

    if IPython.get_ipython() is None:
        IPython.embed()
