## ramp: 16 envs, steady window = last 2.0 s of each phase

| phase | v_cmd | v_meas (ratio) | w_cmd | w_meas (ratio) | b_eff from wheels [m] | |vy| mean [m/s] | z mean / std [mm] | roll std / pitch std [deg] | wheel L/R actual / cmd [rad/s] | moved in window [mm] | moved in last 0.5 s [mm] | max tilt in phase [deg] |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| settle | 0.00 | -0.0001 (nan) | 0.00 | 0.0000 (nan) | nan | 0.0000 | 27.67 / 0.133 | 0.000 / 0.031 | -0.00/-0.00 vs 0.00/0.00 | 0.1 | 0.00 | 0.21 |
| straight_0.10 | 0.10 | 0.0995 (0.995) | 0.00 | -0.0000 (nan) | nan | 0.0000 | 27.71 / 0.000 | 0.000 / 0.000 | 3.57/3.57 vs 3.57/3.57 | 196.9 | 47.74 | 2.59 |
| straight_0.20 | 0.20 | 0.1989 (0.995) | 0.00 | -0.0000 (nan) | nan | 0.0000 | 27.71 / 0.000 | 0.000 / 0.000 | 7.14/7.14 vs 7.14/7.14 | 393.9 | 95.49 | 2.59 |
| stop1 | 0.00 | -0.0472 (nan) | 0.00 | -0.0000 (nan) | nan | 0.0000 | 27.75 / 0.161 | 0.000 / 1.839 | 0.66/0.66 vs 0.64/0.64 | 50.1 | 0.14 | 7.06 |
| spin_1.0 | 0.00 | -0.0001 (nan) | 1.00 | 0.9849 (0.985) | 0.0976 | 0.0000 | 27.73 / 0.000 | 0.000 / 0.000 | -1.72/1.71 vs -1.72/1.72 | 0.2 | 0.07 | 0.34 |
| arc_left | 0.12 | 0.1193 (0.994) | 0.60 | 0.5924 (0.987) | 0.0975 | 0.0008 | 27.71 / 0.000 | 0.000 / 0.000 | 3.25/5.32 vs 3.26/5.32 | 223.0 | 57.09 | 2.12 |
| arc_right | 0.12 | 0.1193 (0.994) | -0.60 | -0.5924 (0.987) | 0.0975 | 0.0008 | 27.71 / 0.000 | 0.000 / 0.000 | 5.32/3.25 vs 5.32/3.26 | 223.0 | 57.13 | 0.26 |
| stop2 | 0.00 | -0.0197 (nan) | 0.00 | 0.1035 (nan) | nan | 0.0010 | 27.73 / 0.085 | 0.052 / 1.330 | 0.11/0.04 vs 0.11/0.04 | 37.7 | 0.00 | 7.17 |

whole run: max |roll| = 2.120 deg, max |pitch| = 7.168 deg, z range = 26.83..31.01 mm, finite = True
env-to-env spread at phase end (x/y mm, yaw deg): settle 0.00/0.00/0.000; straight_0.10 0.00/0.00/0.000; straight_0.20 0.02/0.00/0.001; stop1 0.02/0.00/0.001; spin_1.0 0.04/0.03/0.001; arc_left 0.04/0.03/0.001; arc_right 0.04/0.04/0.001; stop2 0.04/0.04/0.001

wrote ramp_phases.csv, ramp_traj.png, ramp_speeds.png to C:/Users/pjw/ros2camp/pd-newton-check/results/2026-11-21/newton_pinky_check
