"""order 모델 표의 숫자: base 안의 이음매와 base 끝에서 기다린 시간을 따로 센다."""
from fake_robots.world.order_situations import ORDER_HEADER, run_order_situation


def test_a_wait_at_the_base_end_is_counted_apart_from_the_seams_inside_the_base():
    outcome = run_order_situation("normal8", True, 1)
    om = outcome.world.order_metrics
    # 찬 도크 앞의 기다리는 자리가 base 끝이다. 거기서 기다린 시간은 이음매 공백이 아니다
    assert om.base_waits and max(om.base_waits) > 1.0
    # base 안에서는 서지 않는다: 이음매는 세계의 한 걸음(0.1 s)이다. 도크 후진이 마당의 N_W, N_E에서
    # 끝나고 Nav2 goal로 이어지는 곳(추가 지시서 3의 2-1)은 방식이 바뀌어 두 걸음(0.2 s)까지 걸린다
    assert om.seams and max(om.seams) < 0.25
    row = dict(zip(ORDER_HEADER, outcome.row, strict=True))
    assert row["이음매 공백 평균/최대(s)"] == om.seam_text()
    assert om.seam_text().startswith("0.10 / ")
    waits = f"{len(om.base_waits)} / {max(om.base_waits):.1f}"
    assert row["base 끝 기다림(횟수 / 최대 s)"] == waits


def test_with_the_order_model_off_nothing_counts_as_a_wait_at_a_base_end():
    outcome = run_order_situation("normal8", False, 1)
    assert outcome.world.order_metrics.base_waits == []
    row = dict(zip(ORDER_HEADER, outcome.row, strict=True))
    assert row["base 끝 기다림(횟수 / 최대 s)"] == "-"
