# Wheel gain sweep (4 envs each)

kd = wheel velocity-drive damping [N·m·s/rad], eff = wheel effort limit [N·m]. step = instant command
changes, ramp = commands limited to 1.0 m/s² and 3.0 rad/s². Ratios are measured/commanded over the last
2 s of each 3 s phase; max roll/pitch and base z range are over the whole 19 s run (z = base_link height,
wheel centre, nominal 28 mm).

| run | v ratio @0.20 m/s | spin w ratio | arc_left w ratio | max roll [deg] | max pitch [deg] | base z range [mm] |
|---|---|---|---|---|---|---|
| kd0.02_eff0.2_ramp | 0.995 | 0.984 | 1.020 | 0.047 | 0.399 | 26.89..27.74 |
| kd0.02_eff0.2_step | 0.995 | 0.984 | 1.020 | 0.176 | 0.580 | 26.89..27.86 |
| kd0.02_eff0.5_ramp | 0.995 | 0.984 | 1.020 | 0.047 | 0.399 | 26.89..27.74 |
| kd0.02_eff0.5_step | 0.995 | 0.984 | 1.020 | 0.176 | 0.580 | 26.89..27.86 |
| kd0.05_eff0.2_ramp | 0.995 | 0.985 | 0.999 | 0.154 | 0.540 | 26.89..27.85 |
| kd0.05_eff0.2_step | 0.995 | 0.985 | 0.999 | 0.231 | 2.776 | 26.89..28.12 |
| kd0.05_eff0.5_ramp | 0.995 | 0.985 | 0.999 | 0.154 | 0.540 | 26.89..27.85 |
| kd0.05_eff0.5_step | 0.995 | 0.985 | 0.999 | 0.353 | 3.882 | 26.89..28.25 |
| kd0.1_eff0.2_ramp | 0.995 | 0.985 | 0.993 | 0.444 | 0.914 | 26.89..28.25 |
| kd0.1_eff0.2_step | 0.995 | 0.985 | 0.993 | 0.347 | 5.064 | 26.89..28.25 |
| kd0.1_eff0.5_ramp | 0.995 | 0.985 | 0.993 | 0.452 | 0.925 | 26.89..28.26 |
| kd0.1_eff0.5_step | 0.995 | 0.985 | 0.993 | 1.557 | 6.087 | 26.89..29.06 |
| kd0.2_eff0.2_ramp | 0.995 | 0.985 | 0.989 | 0.634 | 1.503 | 26.89..28.22 |
| kd0.2_eff0.2_step | 0.995 | 0.985 | 0.989 | 0.256 | 7.709 | 26.89..28.27 |
| kd0.2_eff0.5_ramp | 0.995 | 0.985 | 0.989 | 1.025 | 2.293 | 26.89..28.98 |
| kd0.2_eff0.5_step | 0.995 | 0.985 | 0.989 | 1.964 | 7.077 | 26.89..29.63 |
| kd0.5_eff0.2_ramp | 0.995 | 0.985 | 0.987 | 2.505 | 8.076 | 26.89..29.96 |
| kd0.5_eff0.2_step | 0.995 | 0.985 | 0.987 | 1.877 | 9.919 | 26.89..30.16 |
| kd0.5_eff0.5_ramp | 0.995 | 0.985 | 0.987 | 2.120 | 7.168 | 26.83..31.01 |
| kd0.5_eff0.5_step | 0.995 | 0.985 | 0.987 | 1.921 | 18.788 | 26.89..30.09 |
