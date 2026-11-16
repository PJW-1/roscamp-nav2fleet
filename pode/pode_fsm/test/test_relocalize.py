"""위치 다시 찾기(relocalize.py)의 순수 로직: 스캔 일치율, 판정, 단계 기계, 바쁨 거절, 시간 초과."""
import math

import pytest
from pode_fsm.params import RelocalizeParams
from pode_fsm.relocalize import (
    BUSY,
    CANCELED,
    GLOBAL,
    NO_LOCALIZER,
    NOMOTION,
    NOT_CONVERGED,
    SCAN_MISMATCH,
    SPIN,
    SPIN_BLOCKED,
    SPIN_TIMEOUT_CODE,
    TIMEOUT,
    CancelSpin,
    Done,
    Estimate,
    GlobalInit,
    MapField,
    Nomotion,
    Relocalizer,
    Spin,
    distance_field,
    failure_reason,
    is_busy,
    is_good,
    nearest_field,
    scan_match,
    scan_points,
    std_from_covariance,
)

RES = 0.05
SIZE = 40  # 2 m x 2 m 방
P = RelocalizeParams()


# ---- 시험 지도와 가짜 스캔 ----
def room():
    """테두리 한 칸이 벽인 2 m 방. 벽 칸 중심선은 0.025와 1.975에 있다."""
    data = [0] * (SIZE * SIZE)
    for i in range(SIZE):
        for j in range(SIZE):
            if i in (0, SIZE - 1) or j in (0, SIZE - 1):
                data[j * SIZE + i] = 100
    return data


def field():
    return MapField(RES, 0.0, 0.0, 0.0,
                    distance_field(RES, SIZE, SIZE, room(), occupied=65, cap_m=0.3))


def fake_scan(x, y, yaw, n=360):
    """(x, y, yaw)에 선 레이저가 벽 칸 중심선까지 잰 거리."""
    lo, hi = 0.025, 1.975
    ranges = []
    for k in range(n):
        a = yaw + (-math.pi + k * 2 * math.pi / n)
        c, s = math.cos(a), math.sin(a)
        ds = []
        if c > 1e-9:
            ds.append((hi - x) / c)
        if c < -1e-9:
            ds.append((lo - x) / c)
        if s > 1e-9:
            ds.append((hi - y) / s)
        if s < -1e-9:
            ds.append((lo - y) / s)
        ranges.append(min(ds))
    return ranges, -math.pi, 2 * math.pi / n


def points(x, y, yaw):
    ranges, a0, inc = fake_scan(x, y, yaw)
    return scan_points(ranges, a0, inc, 0.05, 12.0)


# ---- 거리 변환 ----
def test_distance_field_is_zero_on_walls_and_grows_inward():
    d = distance_field(RES, SIZE, SIZE, room(), occupied=65, cap_m=0.3)
    assert d[0, 5] == 0.0
    assert d[1, 5] == pytest.approx(RES)
    assert d[3, 5] == pytest.approx(3 * RES)
    assert d[3, 3] == pytest.approx(3 * RES)


def test_distance_field_is_infinite_past_the_cap():
    d = distance_field(RES, SIZE, SIZE, room(), occupied=65, cap_m=0.3)
    assert math.isinf(d[20, 20])


def test_unknown_cells_are_not_occupied():
    data = [-1] * (SIZE * SIZE)
    d = distance_field(RES, SIZE, SIZE, data, occupied=65, cap_m=0.3)
    assert math.isinf(d.min())


# ---- 스캔 점 ----
def test_scan_points_drop_invalid_ranges():
    ranges = [1.0, float("nan"), float("inf"), 0.01, 20.0, 2.0]
    pts = scan_points(ranges, 0.0, math.pi / 2, 0.05, 12.0)
    assert len(pts) == 2
    assert tuple(pts[0]) == pytest.approx((1.0, 0.0))
    assert tuple(pts[1]) == pytest.approx((2.0 * math.cos(5 * math.pi / 2),
                                           2.0 * math.sin(5 * math.pi / 2)))


# ---- 스캔 일치율 ----
def test_scan_matches_at_the_true_pose():
    pts = points(1.0, 1.0, 0.3)
    assert scan_match(field(), pts, (1.0, 1.0, 0.3), (0.0, 0.0, 0.0), 0.06) == pytest.approx(1.0)


def test_scan_does_not_match_a_shifted_pose():
    pts = points(1.0, 1.0, 0.0)
    m = scan_match(field(), pts, (1.3, 1.0, 0.0), (0.0, 0.0, 0.0), 0.06)
    assert m < 0.75


def test_scan_does_not_match_a_turned_pose():
    pts = points(1.0, 1.0, 0.0)
    assert scan_match(field(), pts, (1.0, 1.0, 0.4), (0.0, 0.0, 0.0), 0.06) < 0.75


def test_laser_offset_is_applied():
    # 레이저가 로봇 앞 0.1 m에 있다. 로봇 (0.9, 1.0)이면 레이저는 (1.0, 1.0)
    pts = points(1.0, 1.0, 0.0)
    assert scan_match(field(), pts, (0.9, 1.0, 0.0), (0.1, 0.0, 0.0), 0.06) == pytest.approx(1.0)
    assert scan_match(field(), pts, (1.0, 1.0, 0.0), (0.1, 0.0, 0.0), 0.06) < 0.75


def test_points_off_the_map_do_not_match():
    pts = points(1.0, 1.0, 0.0)
    assert scan_match(field(), pts, (10.0, 10.0, 0.0), (0.0, 0.0, 0.0), 0.06) == 0.0


def test_no_valid_points_is_no_match():
    assert scan_match(field(), scan_points([], 0.0, 0.1, 0.05, 12.0),
                      (1.0, 1.0, 0.0), (0.0, 0.0, 0.0), 0.06) == 0.0


def test_map_origin_is_used():
    d = distance_field(RES, SIZE, SIZE, room(), occupied=65, cap_m=0.3)
    moved = MapField(RES, 5.0, -2.0, 0.0, d)
    pts = points(1.0, 1.0, 0.0)
    assert scan_match(moved, pts, (6.0, -1.0, 0.0), (0.0, 0.0, 0.0), 0.06) == pytest.approx(1.0)


# ---- 공분산 ----
def test_std_from_covariance_takes_the_larger_axis_and_yaw():
    cov = [0.0] * 36
    cov[0], cov[7], cov[35] = 0.04, 0.01, 0.0225
    sxy, syaw = std_from_covariance(cov)
    assert sxy == pytest.approx(0.2)
    assert syaw == pytest.approx(0.15)


def test_std_xy_is_the_contract_definition_not_the_eigenvalue():
    # 계약(d8d3123): std_xy_m = sqrt(max(var_x, var_y)). 상관은 보지 않는다
    cov = [0.0] * 36
    cov[0], cov[7] = 0.01, 0.0081
    cov[1] = cov[6] = 0.009
    sxy, _ = std_from_covariance(cov)
    assert sxy == pytest.approx(0.1)


# ---- 판정 ----
def est(sxy=0.05, syaw=0.05, match=0.9):
    return Estimate(1.0, 2.0, 0.5, sxy, syaw, match)


def test_good_needs_small_covariance_and_scan_match():
    assert is_good(est(), P)
    assert not is_good(est(sxy=0.2), P)
    assert not is_good(est(syaw=0.3), P)
    assert not is_good(est(match=0.5), P)


def test_small_covariance_alone_is_not_good():
    assert not is_good(est(sxy=0.001, syaw=0.001, match=0.0), P)


def test_failure_reason():
    assert failure_reason(None, P) == TIMEOUT
    assert failure_reason(est(sxy=0.3, match=0.2), P) == NOT_CONVERGED
    assert failure_reason(est(syaw=0.3), P) == NOT_CONVERGED
    assert failure_reason(est(match=0.4), P) == SCAN_MISMATCH
    assert failure_reason(est(), P) == TIMEOUT  # 좋았지만 K번 이어지지 못했다


# ---- 바쁨 거절 ----
@pytest.mark.parametrize("state", ["MOVING", "DOCKING", "RECOVERING", "BLOCKED"])
def test_busy_while_moving_docking_or_retreating(state):
    assert is_busy(state, relocalizing=False)


@pytest.mark.parametrize("state", ["IDLE", "DOCKED", "ERROR", "LOCALIZING"])
def test_not_busy_when_standing(state):
    assert not is_busy(state, relocalizing=False)


def test_busy_while_another_relocalize_runs():
    assert is_busy("IDLE", relocalizing=True)


# ---- 단계 기계 ----
def run(allow_spin=True, allow_global=False, timeout_s=30.0, ready=True, p=P):
    r = Relocalizer(p, allow_spin, allow_global, timeout_s, now=0.0)
    return r, r.start(ready)


def feed(r, now, e, n):
    cmds = []
    for k in range(n):
        cmds += r.estimate(now + 0.1 * k, e)
    return cmds


def done(cmds):
    found = [c for c in cmds if isinstance(c, Done)]
    assert len(found) == 1, cmds
    return found[0]


def test_missing_localizer_ends_at_once():
    r, cmds = run(ready=False)
    d = done(cmds)
    assert not d.success and d.reason == NO_LOCALIZER
    assert r.stage == ""  # 시작 전 거절이면 단계는 빈칸(계약)


def test_fresh_updates_counts_only_judged_estimates():
    r, _ = run()
    feed(r, 1.0, est(match=0.2), 3)
    assert r.fresh_updates == 3
    r.tick(P.nomotion_share * 30.0)
    feed(r, 8.0, est(), 4)  # 도는 중에는 판정하지 않는다
    assert r.fresh_updates == 3
    r.spin_result(18.0, "succeeded", 0)
    feed(r, 18.1, est(), P.stable_updates)
    assert r.fresh_updates == 3 + P.stable_updates


def test_stage_is_kept_after_done():
    r, _ = run()
    r.tick(P.nomotion_share * 30.0)
    r.spin_result(8.0, "rejected", 0)
    assert r.stage == SPIN


def test_starts_with_a_nomotion_request():
    r, cmds = run()
    assert cmds == [Nomotion()]
    assert r.stage == NOMOTION


def test_nomotion_request_once_per_fresh_scan_and_not_faster_than_the_period():
    r, _ = run()
    assert r.scan(0.05) == []  # 첫 요청 바로 뒤
    assert r.scan(P.nomotion_period_s + 0.01) == [Nomotion()]
    assert r.scan(P.nomotion_period_s + 0.02) == []


def test_success_after_k_stable_good_updates():
    r, _ = run()
    cmds = feed(r, 1.0, est(), P.stable_updates - 1)
    assert not any(isinstance(c, Done) for c in cmds)
    d = done(r.estimate(2.0, est()))
    assert d.success and d.reason == "" and d.estimate == est()


def test_a_bad_update_resets_the_count():
    r, _ = run()
    feed(r, 1.0, est(), P.stable_updates - 1)
    r.estimate(1.5, est(match=0.3))
    cmds = feed(r, 2.0, est(), P.stable_updates - 1)
    assert not any(isinstance(c, Done) for c in cmds)
    assert done(r.estimate(3.0, est())).success


def test_small_covariance_with_scan_mismatch_never_succeeds():
    r, _ = run(allow_spin=False)
    cmds = feed(r, 1.0, est(sxy=0.01, syaw=0.01, match=0.2), 50)
    assert not any(isinstance(c, Done) for c in cmds)
    d = done(r.tick(30.0))
    assert not d.success and d.reason == SCAN_MISMATCH


def test_without_spin_nomotion_gets_the_whole_timeout():
    r, _ = run(allow_spin=False)
    assert r.tick(29.9) == []
    d = done(r.tick(30.0))
    assert d.reason == TIMEOUT  # 추정이 한 번도 오지 않았다


def test_spin_starts_after_the_nomotion_share():
    r, _ = run()
    t = P.nomotion_share * 30.0
    assert r.tick(t - 0.01) == []
    cmds = r.tick(t)
    spins = [c for c in cmds if isinstance(c, Spin)]
    assert len(spins) == 1 and spins[0].yaw == pytest.approx(2 * math.pi)
    assert spins[0].allowance_s == pytest.approx(30.0 - t - P.settle_s)
    assert r.stage == SPIN


def test_updates_during_the_spin_are_not_counted():
    r, _ = run()
    r.tick(P.nomotion_share * 30.0)
    cmds = feed(r, 10.0, est(), P.stable_updates + 2)
    assert not any(isinstance(c, Done) for c in cmds)
    assert r.scan(10.0) == [Nomotion()]  # 도는 중에도 무이동 갱신은 부른다


def test_success_after_the_spin():
    r, _ = run()
    r.tick(P.nomotion_share * 30.0)
    assert r.spin_result(18.0, "succeeded", 0) == []
    assert done(feed(r, 18.1, est(), P.stable_updates)).success


def test_spin_rejected_ends_with_spin_blocked():
    r, _ = run()
    r.tick(P.nomotion_share * 30.0)
    d = done(r.spin_result(8.0, "rejected", 0))
    assert not d.success and d.reason == SPIN_BLOCKED


def test_spin_collision_ends_with_spin_blocked():
    r, _ = run()
    r.tick(P.nomotion_share * 30.0)
    d = done(r.spin_result(9.0, "aborted", 703))
    assert d.reason == SPIN_BLOCKED


def test_spin_timeout_is_not_blocked_and_is_evaluated():
    r, _ = run()
    r.tick(P.nomotion_share * 30.0)
    assert r.spin_result(25.0, "aborted", SPIN_TIMEOUT_CODE) == []
    assert done(feed(r, 25.1, est(), P.stable_updates)).success


def test_spin_failure_with_global_goes_global_without_spinning():
    r, _ = run(allow_global=True)
    r.tick(P.nomotion_share * 30.0)
    cmds = r.spin_result(9.0, "aborted", 703)
    assert cmds == [GlobalInit()]
    assert r.stage == GLOBAL
    assert r.global_ready(9.2, True) == []  # 돌 수 없었으니 돌지 않는다
    assert done(feed(r, 9.5, est(), P.stable_updates)).success


def test_global_stage_after_the_spin_share():
    r, _ = run(allow_global=True)
    r.tick(P.nomotion_share * 30.0)
    r.spin_result(15.0, "succeeded", 0)
    t = (P.nomotion_share + P.spin_share) * 30.0
    assert r.tick(t - 0.01) == []
    assert r.tick(t) == [GlobalInit()]
    assert r.stage == GLOBAL


def test_global_ignores_updates_until_reinit_answered_then_spins():
    r, _ = run(allow_global=True)
    r.tick(P.nomotion_share * 30.0)
    r.spin_result(15.0, "succeeded", 0)
    t = (P.nomotion_share + P.spin_share) * 30.0
    r.tick(t)
    assert not any(isinstance(c, Done) for c in feed(r, t + 0.1, est(), P.stable_updates + 1))
    assert r.scan(t + 0.5) == []  # 재초기화 답 전에는 갱신을 부르지 않는다
    cmds = r.global_ready(t + 0.6, True)
    spins = [c for c in cmds if isinstance(c, Spin)]
    assert len(spins) == 1
    assert spins[0].allowance_s == pytest.approx(30.0 - (t + 0.6) - P.settle_s)
    r.spin_result(29.0, "succeeded", 0)
    assert done(feed(r, 29.0, est(), P.stable_updates)).success


def test_global_reinit_failure_is_no_localizer():
    r, _ = run(allow_spin=False, allow_global=True)
    r.tick(P.nomotion_share * 30.0)
    d = done(r.global_ready(9.0, False))
    assert d.reason == NO_LOCALIZER


def test_without_spin_global_follows_nomotion():
    r, _ = run(allow_spin=False, allow_global=True)
    assert r.tick(P.nomotion_share * 30.0) == [GlobalInit()]


def test_timeout_while_spinning_cancels_the_spin():
    r, _ = run()
    r.tick(P.nomotion_share * 30.0)
    cmds = r.tick(30.0)
    assert CancelSpin() in cmds
    d = done(cmds)
    assert not d.success and d.reason == TIMEOUT


def test_timeout_reports_the_last_estimate():
    r, _ = run(allow_spin=False)
    r.estimate(1.0, est(sxy=0.5))
    d = done(r.tick(30.0))
    assert d.reason == NOT_CONVERGED and d.estimate == est(sxy=0.5)


def test_cancel_stops_the_spin_and_ends_canceled():
    r, _ = run()
    r.tick(P.nomotion_share * 30.0)
    cmds = r.cancel(10.0)
    assert CancelSpin() in cmds
    assert done(cmds).reason == CANCELED


def test_nothing_after_done():
    r, _ = run()
    feed(r, 1.0, est(), P.stable_updates)
    assert r.tick(40.0) == [] and r.scan(40.0) == [] and r.cancel(40.0) == []
    assert r.estimate(40.0, est()) == []


def test_zero_timeout_uses_the_default():
    r, _ = run(allow_spin=False, timeout_s=0.0)
    assert r.tick(P.default_timeout_s - 0.01) == []
    assert done(r.tick(P.default_timeout_s)).reason == TIMEOUT


def test_busy_reason_name():
    assert BUSY == "BUSY"


# ---- Codex 938df90 지적 반영 ----
def test_point_near_a_wall_cell_is_measured_to_the_cell_not_its_centre():
    """칸 중심 사이 거리(0.10 m)가 아니라 점에서 벽 칸까지 거리(0.055 m)로 잰다."""
    f = MapField(RES, 0.0, 0.0, 0.0, *nearest_field(RES, SIZE, SIZE, room(), 65, 0.2))
    pts = scan_points([0.895], math.pi, 1.0, 0.05, 12.0)  # 레이저 (1.0, 1.0)에서 왼쪽 → x=0.105
    assert scan_match(f, pts, (1.0, 1.0, 0.0), (0.0, 0.0, 0.0), 0.06) == 1.0
    pts = scan_points([0.885], math.pi, 1.0, 0.05, 12.0)  # x=0.115 → 0.065 m
    assert scan_match(f, pts, (1.0, 1.0, 0.0), (0.0, 0.0, 0.0), 0.06) == 0.0


def test_nearest_field_matches_distance_field():
    d = distance_field(RES, SIZE, SIZE, room(), occupied=65, cap_m=0.3)
    nd, ni, nj = nearest_field(RES, SIZE, SIZE, room(), 65, 0.3)
    assert (nd == d).all()
    assert (ni[5, 3], nj[5, 3]) == (0, 5)  # 왼쪽 벽 칸
    assert ni[20, 20] == -1


def test_estimate_after_the_deadline_cannot_succeed():
    r, _ = run(allow_spin=False)
    feed(r, 1.0, est(), P.stable_updates - 1)
    assert r.estimate(30.5, est()) == []
    d = done(r.tick(30.5))
    assert not d.success and d.reason == TIMEOUT


def test_spin_allowance_is_never_zero():
    """Jazzy Spin은 0을 무제한으로 본다. 몫이 모자라도 최소값을 준다."""
    p = RelocalizeParams(settle_s=25.0)  # 30 - 7.5 - 25 < 0
    r = Relocalizer(p, True, False, 30.0, now=0.0)
    r.start(True)
    spins = [c for c in r.tick(p.nomotion_share * 30.0) if isinstance(c, Spin)]
    assert spins[0].allowance_s == pytest.approx(p.min_spin_allowance_s)
