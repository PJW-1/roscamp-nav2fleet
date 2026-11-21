# main 쪽 실패 두 판의 근거 (launch.out, truth.csv에서 뽑음)

판 원본: WSL `~/gz_runs/main_a6265b0_v012/v7b`, `v7d`. 시각은 줄 앞의 벽시계(초), truth의 sim 시각은 시뮬 초다.

## v7b

```
watchdog: ROBOT_ISOLATED, Ctrl+C at 11:17:13
[INFO] [1795227277.954054024] [sim_stand_in]: pode1: dock OUTBOUND ok (5.0, 0.0) cm
[INFO] [1795227298.405760201] [sim_stand_in]: pode2: dock OUTBOUND ok (6.7, -1.3) cm
[INFO] [1795227310.538506975] [pode2.pode_fsm]: state MOVING -> RECOVERING wait (self-recovery attempts used 1)
[INFO] [1795227311.109610176] [fleet_manager]: SEGMENT_DONE pode3-25 seq 1 (follow_path) to O_X
[INFO] [1795227312.597010077] [pode2.pode_fsm]: state RECOVERING -> BLOCKED  (self-recovery attempts used 1)
[WARN] [1795227322.778620377] [fleet_manager]: ZONE_MARKED_BLOCKED here->Q2 in HALL, tried again in 60 s
[WARN] [1795227327.881321651] [fleet_manager]: NO_START move goal 34 to Q2: moved less than 0.02 m in 5 s
[WARN] [1795227333.903063123] [fleet_manager]: GOAL_RETRY NAV_ABORTED, attempt 1
[WARN] [1795227341.013394720] [fleet_manager]: NO_START move goal 35 to Q2: moved less than 0.02 m in 5 s
[WARN] [1795227356.086651224] [fleet_manager]: ZONE_MARKED_BLOCKED here->E-3 in HALL, tried again in 60 s
[WARN] [1795227360.908348318] [fleet_manager]: GOAL_RETRY NAV_ABORTED, attempt 2
[WARN] [1795227368.066021321] [fleet_manager]: NO_START move goal 37 to Q2: moved less than 0.02 m in 5 s
[WARN] [1795227383.126949756] [fleet_manager]: ZONE_MARKED_BLOCKED O_OUT->E-3 in HALL, tried again in 60 s
[ERROR] [1795227387.929277477] [fleet_manager]: GOAL_FAILED NAV_ABORTED after 3 attempts
[ERROR] [1795227387.929829277] [fleet_manager]: ROBOT_ISOLATED NAV_ABORTED; holds ['HALL', 'OUT_DOCK']
sim_time=83.828 robot=pode2 event=start episode=1 state=RECOVERING truth_x=0.190 truth_y=0.741 truth_yaw=-1.751 amcl_x=0.175 amcl_y=0.752 amcl_yaw=-1.789 err_m=0.019
```

## v7d

```
watchdog: ROBOT_ISOLATED, Ctrl+C at 11:34:38
[INFO] [1795228356.632007360] [sim_stand_in]: pode1: dock OUTBOUND ok (1.8, 0.2) cm
[INFO] [1795228365.636586174] [pode2.pode_fsm]: state MOVING -> RECOVERING wait (self-recovery attempts used 1)
[INFO] [1795228377.337167814] [sim_stand_in]: pode2: dock OUTBOUND ok (5.8, -1.2) cm
[INFO] [1795228389.222573644] [fleet_manager]: SEGMENT_DONE pode3-25 seq 1 (follow_path) to O_X
[WARN] [1795228400.621501595] [fleet_manager]: OUTSIDE_ZONE at (0.59, 0.85), 0.09 m outside ['OUT_DOCK'] for 5 reports
[WARN] [1795228411.221795724] [fleet_manager]: OUTSIDE_ZONE at (0.69, 0.94), 0.19 m outside ['OUT_DOCK'] for 5 reports
[INFO] [1795228419.149634842] [fleet_manager]: SEGMENT_DONE pode3-25 seq 2 (follow_path) to O
[INFO] [1795228422.171350148] [sim_stand_in]: pode3: dock OUTBOUND FAILED (103.3, -2.4) cm
[WARN] [1795228422.247964748] [fleet_manager]: GOAL_RETRY DOCK_FAILED, attempt 1
[INFO] [1795228427.258556355] [sim_stand_in]: pode3: dock OUTBOUND FAILED (103.3, -2.4) cm
[WARN] [1795228427.348371955] [fleet_manager]: GOAL_RETRY DOCK_FAILED, attempt 2
[INFO] [1795228432.393986862] [sim_stand_in]: pode3: dock OUTBOUND FAILED (103.3, -2.4) cm
[ERROR] [1795228432.463780062] [fleet_manager]: GOAL_FAILED DOCK_FAILED after 3 attempts
[ERROR] [1795228432.464424562] [fleet_manager]: ROBOT_ISOLATED DOCK_FAILED; holds ['OUT_DOCK']
[INFO] [1795228432.739201862] [pode2.pode_fsm]: state MOVING -> RECOVERING wait (self-recovery attempts used 1)
sim_time=60.436 robot=pode2 event=start episode=1 state=RECOVERING truth_x=0.528 truth_y=0.608 truth_yaw=2.632 amcl_x=0.533 amcl_y=0.604 amcl_yaw=2.611 err_m=0.007
sim_time=127.480 robot=pode2 event=start episode=2 state=RECOVERING truth_x=0.534 truth_y=0.602 truth_yaw=2.662 amcl_x=0.532 amcl_y=0.601 amcl_yaw=2.661 err_m=0.002
```

## 위치 기록(truth.csv, 2 s마다)

v7b pode2(78~92 s), v7b pode1·pode3(82~86 s), v7d pode3(80~130 s):

```
v7b pode2
sim 78.0 (0.35,0.85,-2.99)
sim 80.0 (0.34,0.85,-3.00)
sim 82.0 (0.21,0.79,-2.14)
sim 84.1 (0.19,0.74,-1.75)
sim 86.1 (0.19,0.74,-1.75)
sim 88.1 (0.19,0.74,-1.75)
sim 90.1 (0.19,0.74,-1.75)
v7b pode1
sim 82.0 (1.86,0.37,1.62)
sim 84.1 (1.85,0.46,1.65)
v7b pode3
sim 82.0 (0.98,0.87,3.13)
sim 84.1 (0.75,0.87,-3.12)
v7d pode3
sim 80.0 (1.53,0.83,-1.48)
sim 82.0 (1.52,0.83,-1.38)
sim 84.1 (1.52,0.84,-1.57)
sim 86.1 (1.52,0.84,-2.03)
sim 88.1 (1.53,0.84,-2.09)
sim 90.1 (1.53,0.84,-1.18)
sim 92.1 (1.53,0.85,-0.95)
sim 94.1 (1.54,0.85,-2.26)
sim 96.2 (1.51,0.85,-1.01)
sim 98.2 (1.51,0.83,-1.87)
sim 100.2 (1.45,0.83,-1.57)
sim 102.2 (1.39,0.84,-1.75)
sim 104.2 (1.34,0.83,-1.61)
sim 106.2 (1.32,0.83,-1.57)
sim 108.3 (1.32,0.83,-1.55)
sim 110.3 (1.32,0.83,-1.55)
sim 112.3 (1.32,0.83,-1.55)
sim 114.3 (1.32,0.84,-1.87)
sim 116.3 (1.32,0.84,-1.87)
sim 118.3 (1.32,0.84,-1.87)
sim 120.3 (1.32,0.84,-1.87)
sim 122.4 (1.32,0.84,-1.87)
sim 124.4 (1.32,0.84,-1.87)
sim 126.4 (1.32,0.84,-1.87)
sim 128.4 (1.32,0.84,-1.87)
```
