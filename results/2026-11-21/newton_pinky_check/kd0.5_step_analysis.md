## step: 16 envs, steady window = last 2.0 s of each phase

| phase | v_cmd | v_meas (ratio) | w_cmd | w_meas (ratio) | b_eff from wheels [m] | |vy| mean [m/s] | z mean / std [mm] | roll std / pitch std [deg] | wheel L/R actual / cmd [rad/s] | moved in window [mm] | moved in last 0.5 s [mm] | max tilt in phase [deg] |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| settle | 0.00 | -0.0001 (nan) | 0.00 | 0.0000 (nan) | nan | 0.0000 | 27.67 / 0.133 | 0.000 / 0.031 | -0.00/-0.00 vs 0.00/0.00 | 0.1 | 0.00 | 0.21 |
| straight_0.10 | 0.10 | 0.0995 (0.995) | 0.00 | 0.0000 (nan) | nan | 0.0000 | 27.71 / 0.000 | 0.000 / 0.000 | 3.57/3.57 vs 3.57/3.57 | 196.9 | 47.74 | 0.66 |
| straight_0.20 | 0.20 | 0.1989 (0.995) | 0.00 | 0.0000 (nan) | nan | 0.0000 | 27.71 / 0.000 | 0.000 / 0.000 | 7.14/7.14 vs 7.14/7.14 | 393.9 | 95.49 | 0.66 |
| stop1 | 0.00 | -0.0493 (nan) | 0.00 | 0.0042 (nan) | nan | 0.0012 | 27.85 / 0.523 | 0.169 / 4.808 | 0.32/0.32 vs 0.14/0.14 | 54.4 | 0.11 | 18.79 |
| spin_1.0 | 0.00 | -0.0001 (nan) | 1.00 | 0.9849 (0.985) | 0.0976 | 0.0000 | 27.73 / 0.000 | 0.000 / 0.000 | -1.72/1.71 vs -1.72/1.72 | 0.2 | 0.07 | 0.38 |
| arc_left | 0.12 | 0.1193 (0.994) | 0.60 | 0.5924 (0.987) | 0.0975 | 0.0008 | 27.71 / 0.000 | 0.000 / 0.000 | 3.25/5.32 vs 3.26/5.32 | 223.0 | 57.13 | 1.92 |
| arc_right | 0.12 | 0.1193 (0.994) | -0.60 | -0.5924 (0.987) | 0.0975 | 0.0008 | 27.71 / 0.000 | 0.000 / 0.000 | 5.32/3.25 vs 5.32/3.26 | 223.0 | 57.13 | 0.41 |
| stop2 | 0.00 | -0.0085 (nan) | 0.00 | -0.0061 (nan) | nan | 0.0005 | 27.72 / 0.073 | 0.051 / 1.129 | 0.04/0.01 vs 0.00/0.00 | 14.5 | 0.01 | 6.89 |

whole run: max |roll| = 1.921 deg, max |pitch| = 18.788 deg, z range = 26.89..30.09 mm, finite = True
env-to-env spread at phase end (x/y mm, yaw deg): settle 0.00/0.00/0.000; straight_0.10 0.00/0.00/0.000; straight_0.20 0.03/0.00/0.001; stop1 0.03/0.00/0.001; spin_1.0 0.04/0.02/0.001; arc_left 0.05/0.03/0.001; arc_right 0.05/0.03/0.001; stop2 0.08/0.07/0.001

wrote step_phases.csv, step_traj.png, step_speeds.png to C:/Users/pjw/ros2camp/pd-newton-check/results/2026-11-21/newton_pinky_check
