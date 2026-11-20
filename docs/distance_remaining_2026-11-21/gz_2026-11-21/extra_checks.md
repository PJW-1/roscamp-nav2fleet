lead main_a6265b0 rush: n 48 median 0.50 mean 0.30 min -5.19 max 0.70
lead main_a6265b0 v7: n 32 median 0.50 mean 0.53 min 0.50 max 0.70
lead dist_fcf3cd0 rush: n 48 median 0.30 mean 0.14 min -5.90 max 0.51
lead dist_fcf3cd0 v7: n 32 median 0.50 mean 0.42 min 0.30 max 0.70
jumps main_a6265b0 rush1: 0 by segment mode {}
jumps main_a6265b0 rush2: 0 by segment mode {}
jumps main_a6265b0 rush3: 0 by segment mode {}
jumps main_a6265b0 rush4: 0 by segment mode {}
jumps main_a6265b0 v7a: 0 by segment mode {}
jumps main_a6265b0 v7b: 0 by segment mode {}
jumps main_a6265b0 v7c: 0 by segment mode {}
jumps main_a6265b0 v7d: 0 by segment mode {}
jumps dist_fcf3cd0 rush1: 15 by segment mode {'follow_path': 12, 'nav_goal': 3}
jump dist_fcf3cd0 rush1 t 48.5 pode1 pode1-19 seq 2 1.586 -> 1.763 m, straight 1.588 m, old value usable True, 0.2 s after the first value of this seq
jump dist_fcf3cd0 rush1 t 123.6 pode1 pode1-59 seq 2 0.065 -> 1.245 m, straight 1.198 m, old value usable False, 0.8 s after the first value of this seq
jump dist_fcf3cd0 rush1 t 208.2 pode3 pode3-104 seq 2 0.053 -> 1.235 m, straight 1.196 m, old value usable False, 1.0 s after the first value of this seq
jumps dist_fcf3cd0 rush2: 16 by segment mode {'follow_path': 13, 'nav_goal': 3}
jump dist_fcf3cd0 rush2 t 17.0 pode2 pode2-4 seq 2 1.342 -> 1.500 m, straight 1.281 m, old value usable True, 1.0 s after the first value of this seq
jump dist_fcf3cd0 rush2 t 28.6 pode3 pode3-10 seq 2 1.541 -> 1.643 m, straight 1.541 m, old value usable True, 0.2 s after the first value of this seq
jump dist_fcf3cd0 rush2 t 123.8 pode1 pode1-59 seq 2 0.126 -> 1.388 m, straight 1.250 m, old value usable False, 0.2 s after the first value of this seq
jumps dist_fcf3cd0 rush3: 16 by segment mode {'follow_path': 14, 'nav_goal': 2}
jump dist_fcf3cd0 rush3 t 17.3 pode2 pode2-4 seq 2 1.341 -> 1.450 m, straight 1.281 m, old value usable True, 1.0 s after the first value of this seq
jump dist_fcf3cd0 rush3 t 209.1 pode3 pode3-105 seq 2 0.088 -> 1.271 m, straight 1.221 m, old value usable False, 0.6 s after the first value of this seq
jumps dist_fcf3cd0 rush4: 15 by segment mode {'follow_path': 13, 'nav_goal': 2}
jump dist_fcf3cd0 rush4 t 122.5 pode1 pode1-59 seq 2 0.113 -> 1.296 m, straight 1.242 m, old value usable False, 0.6 s after the first value of this seq
jump dist_fcf3cd0 rush4 t 207.9 pode3 pode3-104 seq 2 0.065 -> 1.253 m, straight 1.212 m, old value usable False, 1.0 s after the first value of this seq
jumps dist_fcf3cd0 v7a: 15 by segment mode {'follow_path': 15}
jumps dist_fcf3cd0 v7b: 15 by segment mode {'follow_path': 15}
jumps dist_fcf3cd0 v7c: 16 by segment mode {'follow_path': 15, 'nav_goal': 1}
jump dist_fcf3cd0 v7c t 60.9 pode1 pode1-25 seq 2 1.529 -> 1.641 m, straight 1.529 m, old value usable True, 0.2 s after the first value of this seq
jumps dist_fcf3cd0 v7d: 16 by segment mode {'follow_path': 15, 'nav_goal': 1}
jump dist_fcf3cd0 v7d t 69.2 pode2 pode2-27 seq 2 0.084 -> 0.488 m, straight 0.059 m, old value usable True, 2.1 s after the first value of this seq

speed fit main_a6265b0: 1848 samples
| speed m/s | proj mean abs | proj p90 | proj mean | used mean abs | used p90 | used mean |
|---|---|---|---|---|---|---|
| 0.12 | 2.94 | 5.10 | +1.83 | 2.94 | 5.10 | +1.83 |
| 0.14 | 2.21 | 3.46 | +0.98 | 2.21 | 3.46 | +0.98 |
| 0.16 | 1.69 | 2.84 | +0.34 | 1.69 | 2.84 | +0.34 |
| 0.17 | 1.49 | 2.90 | +0.08 | 1.49 | 2.90 | +0.08 |
| 0.18 | 1.34 | 2.98 | -0.15 | 1.34 | 2.98 | -0.15 |
| 0.19 | 1.22 | 3.04 | -0.36 | 1.22 | 3.04 | -0.36 |
| 0.20 | 1.14 | 3.11 | -0.55 | 1.14 | 3.11 | -0.55 |
| 0.22 | 1.09 | 3.22 | -0.87 | 1.09 | 3.22 | -0.87 |
best proj: 0.214 m/s, mean abs 1.08 s
  0-1 s: n 219 mean abs 0.45 p90 0.65 mean +0.15
  1-2 s: n 241 mean abs 0.76 p90 1.56 mean -0.25
  2-4 s: n 476 mean abs 1.19 p90 2.95 mean -0.80
  4-8 s: n 771 mean abs 1.16 p90 3.63 mean -1.01
  8-inf s: n 141 mean abs 1.78 p90 3.04 mean -1.78
best used: 0.214 m/s, mean abs 1.08 s
  0-1 s: n 219 mean abs 0.45 p90 0.65 mean +0.15
  1-2 s: n 241 mean abs 0.76 p90 1.56 mean -0.25
  2-4 s: n 476 mean abs 1.19 p90 2.95 mean -0.80
  4-8 s: n 771 mean abs 1.16 p90 3.63 mean -1.01
  8-inf s: n 141 mean abs 1.78 p90 3.04 mean -1.78

speed fit dist_fcf3cd0: 1975 samples
| speed m/s | proj mean abs | proj p90 | proj mean | used mean abs | used p90 | used mean |
|---|---|---|---|---|---|---|
| 0.12 | 2.81 | 5.01 | +1.72 | 3.15 | 5.87 | +2.17 |
| 0.14 | 2.12 | 3.40 | +0.90 | 2.40 | 4.08 | +1.28 |
| 0.16 | 1.62 | 2.78 | +0.28 | 1.86 | 2.91 | +0.61 |
| 0.17 | 1.44 | 2.87 | +0.02 | 1.66 | 2.71 | +0.33 |
| 0.18 | 1.30 | 2.92 | -0.20 | 1.50 | 2.80 | +0.09 |
| 0.19 | 1.20 | 3.01 | -0.41 | 1.37 | 2.88 | -0.13 |
| 0.20 | 1.12 | 3.08 | -0.59 | 1.26 | 2.95 | -0.33 |
| 0.22 | 1.10 | 3.20 | -0.91 | 1.13 | 3.08 | -0.67 |
best proj: 0.212 m/s, mean abs 1.08 s
  0-1 s: n 243 mean abs 0.42 p90 0.58 mean +0.16
  1-2 s: n 255 mean abs 0.73 p90 1.48 mean -0.26
  2-4 s: n 512 mean abs 1.18 p90 2.93 mean -0.82
  4-8 s: n 831 mean abs 1.21 p90 3.60 mean -1.05
  8-inf s: n 134 mean abs 1.74 p90 2.84 mean -1.74
best used: 0.228 m/s, mean abs 1.12 s
  0-1 s: n 243 mean abs 0.49 p90 0.71 mean +0.23
  1-2 s: n 255 mean abs 0.76 p90 1.45 mean -0.20
  2-4 s: n 512 mean abs 1.19 p90 2.87 mean -0.78
  4-8 s: n 831 mean abs 1.24 p90 3.55 mean -1.08
  8-inf s: n 134 mean abs 1.94 p90 3.20 mean -1.94
used / proj where used came from distance (proj > 0.5 s): median 1.084, n 1649
