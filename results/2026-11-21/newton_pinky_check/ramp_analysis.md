## ramp: 16 envs, steady window = last 2.0 s of each phase

| phase | v_cmd | v_meas (ratio) | w_cmd | w_meas (ratio) | b_eff from wheels [m] | |vy| mean [m/s] | z mean / std [mm] | roll std / pitch std [deg] | wheel L/R actual / cmd [rad/s] | moved in window [mm] | moved in last 0.5 s [mm] | max tilt in phase [deg] |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| settle | 0.00 | -0.0001 (nan) | 0.00 | 0.0000 (nan) | nan | 0.0000 | 27.67 / 0.133 | 0.000 / 0.030 | -0.00/-0.00 vs 0.00/0.00 | 0.1 | 0.00 | 0.21 |
| straight_0.10 | 0.10 | 0.0995 (0.995) | 0.00 | 0.0000 (nan) | nan | 0.0000 | 27.71 / 0.000 | 0.000 / 0.000 | 3.57/3.57 vs 3.57/3.57 | 196.9 | 47.74 | 0.27 |
| straight_0.20 | 0.20 | 0.1989 (0.995) | 0.00 | -0.0000 (nan) | nan | 0.0000 | 27.71 / 0.000 | 0.000 / 0.000 | 7.14/7.14 vs 7.14/7.14 | 393.9 | 95.49 | 0.27 |
| stop1 | 0.00 | 0.0133 (nan) | 0.00 | -0.0000 (nan) | nan | 0.0000 | 27.71 / 0.009 | 0.000 / 0.018 | 0.72/0.72 vs 0.64/0.64 | 10.7 | 0.00 | 0.27 |
| spin_1.0 | 0.00 | -0.0005 (nan) | 1.00 | 0.9848 (0.985) | 0.0976 | 0.0000 | 27.73 / 0.000 | 0.000 / 0.001 | -1.73/1.70 vs -1.72/1.72 | 0.9 | 0.25 | 0.34 |
| arc_left | 0.12 | 0.1192 (0.994) | 0.60 | 0.5996 (0.999) | 0.0975 | 0.0008 | 27.71 / 0.000 | 0.000 / 0.000 | 3.24/5.33 vs 3.26/5.32 | 222.5 | 57.04 | 0.54 |
| arc_right | 0.12 | 0.1192 (0.994) | -0.60 | -0.5996 (0.999) | 0.0975 | 0.0008 | 27.71 / 0.000 | 0.000 / 0.000 | 5.33/3.24 vs 5.32/3.26 | 222.5 | 57.09 | 0.21 |
| stop2 | 0.00 | 0.0005 (nan) | 0.00 | -0.0152 (nan) | nan | 0.0000 | 27.71 / 0.005 | 0.001 / 0.012 | 0.14/0.05 vs 0.11/0.04 | 0.3 | 0.01 | 0.26 |

whole run: max |roll| = 0.154 deg, max |pitch| = 0.540 deg, z range = 26.89..27.85 mm, finite = True
env-to-env spread at phase end (x/y mm, yaw deg): settle 0.00/0.00/0.000; straight_0.10 0.00/0.00/0.000; straight_0.20 0.03/0.00/0.000; stop1 0.03/0.00/0.000; spin_1.0 0.03/0.00/0.000; arc_left 0.03/0.00/0.001; arc_right 0.03/0.01/0.001; stop2 0.07/0.04/0.001

wrote ramp_phases.csv, ramp_traj.png, ramp_speeds.png to C:/Users/pjw/ros2camp/pd-newton-check/results/2026-11-21/newton_pinky_check
