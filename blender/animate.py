import os
from os.path import dirname as up

import bpy
import numpy as np

# Global parameters
FPS = 20
PATH = up(up(up(os.path.realpath(__file__))))

# Simulation time parameters
N = int(np.genfromtxt(PATH + '/data/sim_params.csv', delimiter=','))
bpy.data.scenes['Scene'].frame_start = 0
bpy.data.scenes['Scene'].frame_end = N*2//FPS

# Load simulation data
state = {}
trajectory = {}
state['position'] = np.genfromtxt(PATH + '/data/state_position.csv', delimiter=',').T.reshape(N, 2, 3)
state['angle'] = np.genfromtxt(PATH + '/data/state_angle.csv', delimiter=',').T.reshape(N, 2, 3)
trajectory['position'] = np.genfromtxt(PATH + '/data/trajectory_position.csv', delimiter=',').T.reshape(N, 5, 3)
trajectory['angle'] = np.genfromtxt(PATH + '/data/trajectory_angle.csv', delimiter=',').T.reshape(N, 3, 3)

# Setup simulation
frame = 0
quad = bpy.data.objects['Quadcopter']
bpy.context.active_object.animation_data_clear()
scale_factor = 1.0 * 0.001

# Run simulation
for i in range(N):
    if i%(FPS//2) == 0:
        bpy.context.scene.frame_set(frame)
        # Scale XYZ for visualization
        if i == 0:
            quad.scale = (scale_factor, scale_factor, scale_factor)
            quad.keyframe_insert(data_path='scale', index=-1)
        
        # Translate XYZ
        quad.location = tuple(state['position'][i, 0, :])
        quad.keyframe_insert(data_path='location', index=-1)

        # Rotate ZYX
        quad.rotation_euler = tuple(state['angle'][i, 0, :])
        quad.keyframe_insert(data_path='rotation_euler', index=-1)
        frame += 1

# Update path for display
bpy.ops.object.paths_update()

# Save blender file
bpy.ops.wm.save_mainfile()

# Play animation
bpy.ops.screen.frame_jump(end=False)
bpy.ops.screen.animation_play()
