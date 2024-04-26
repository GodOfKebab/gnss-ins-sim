# -*- coding: utf-8 -*-
# Filename: demo_no_algo.py

"""
The simplest demo of Sim.
Only generate reference trajectory (pos, vel, sensor output). No algorithm.
Created on 2018-01-23
@author: dongxiaoguang
"""

import os
import math
from gnss_ins_sim.sim import imu_model
from gnss_ins_sim.sim import ins_sim

# globals
D2R = math.pi/180

motion_def_path = os.path.abspath('.//demo_motion_def_files//')
fs = 100.0          # IMU sample frequency
fs_gps = 10.0       # GPS sample frequency
fs_mag = fs         # magnetometer sample frequency, not used for now

motion_def_name = "motion_def-0to100"

def test_path_gen():
    '''
    test only path generation in Sim.
    '''
    #### choose a built-in IMU model, typical for IMU381
    imu_err = 'mid-accuracy'
    # generate GPS and magnetometer data
    imu = imu_model.IMU(accuracy=imu_err, axis=9, gps=True)

    #### start simulation
    sim = ins_sim.Sim([fs, fs_gps, fs_mag],
                      os.path.join(motion_def_path, f"{motion_def_name}.csv"),
                      ref_frame=1,
                      imu=imu,
                      mode=None,
                      env=None,
                      algorithm=None)
    sim.run(1)
    # save simulation data to files
    sim.results('')
    acc = sim.dmgr.get_data(["accel"])[0][0]
    print(f"{acc.shape=}")

    gyro = sim.dmgr.get_data(["gyro"])[0][0]
    print(f"{gyro.shape=}")

    pos = sim.dmgr.get_data(["ref_pos"])[0]
    print(f"{pos.shape=} {pos[:, 2].min()=} {pos[:, 2].max()=}")

    rot = sim.dmgr.get_data(["ref_att_euler"])[0]
    print(f"{rot.shape=}")

    time = sim.dmgr.get_data(["time"])[0]
    print(f"{time.shape=}")

    output_folder = os.path.abspath(f"output")
    os.makedirs(output_folder, exist_ok=True)
    with open(os.path.join(output_folder, f"{motion_def_name.lstrip('motion_def-')}.csv"), "w") as f:
        f.write(f"ax (m/s^2), ay (m/s^2), az (m/s^2), gx (deg/s), gy (deg/s), gz (deg/s), ")
        f.write(f"px (m), py (m), pz (m), rx (roll-deg), ry (pitch-deg), rz (yaw-deg), t (sec)\n")
        for i in range(acc.shape[0]):
            ax, ay, az = acc[i, 0], acc[i, 1], acc[i, 2]
            gx, gy, gz = gyro[i, 0], gyro[i, 1], gyro[i, 2]
            px, py, pz = pos[i, 0] - pos[max(0, i-1), 0], pos[i, 1] - pos[max(0, i-1), 1], pos[i, 2] - pos[max(0, i-1), 2]
            rx, ry, rz = rot[i, 2] - rot[max(0, i-1), 2], rot[i, 1] - rot[max(0, i-1), 1], rot[i, 0] - rot[max(0, i-1), 0]
            t = time[i]
            f.write(f"{ax}, {ay}, {az}, {gx}, {gy}, {gz}, {px}, {py}, {pz}, {rx}, {ry}, {rz}, {t}\n")

    # plot data, 3d plot of reference positoin, 2d plots of gyro and accel
    sim.plot(['ref_pos', 'gyro', 'gps_visibility'], opt={'ref_pos': '3d'})

if __name__ == '__main__':
    test_path_gen()
