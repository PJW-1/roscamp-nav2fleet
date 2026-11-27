"""담기 전 결품 확인의 상태: 모자란 수, 답 맞추기, 3초 한도, 버리기 (W-40 · IR-40)."""
from types import SimpleNamespace

from fleet_manager.stock_check import LIMIT_S, StockChecks, shortfall


def fill(goal_id=7, items=None):
    return SimpleNamespace(goal_id=goal_id, items=items or {"choco_bar": 2})


class Pending:
    def __init__(self, done=False):
        self._done = done

    def done(self):
        return self._done


def test_enough_stock_is_no_shortfall():
    assert shortfall({"choco_bar": 2}, {"choco_bar": 3}) == {}


def test_a_short_kind_counts_only_what_is_missing():
    assert shortfall({"choco_bar": 2, "mentos": 1}, {"choco_bar": 1, "mentos": 1}) == {
        "choco_bar": 1}


def test_a_kind_the_answer_leaves_out_has_none():
    assert shortfall({"choco_bar": 2}, {}) == {"choco_bar": 2}


def test_only_the_awaited_answer_gives_the_check_back():
    checks, future = StockChecks(), Pending()
    checks.ask(fill(), future, 0.0)
    assert checks.answered(7, Pending()) is None
    check = checks.answered(7, future)
    assert (check.command.goal_id, check.asked_at) == (7, 0.0)
    assert 7 not in checks and checks.answered(7, future) is None


def test_an_answer_that_already_arrived_is_not_expired():
    checks = StockChecks()
    checks.ask(fill(), Pending(done=True), 0.0)
    assert checks.expired(LIMIT_S + 5.0) == [] and 7 in checks


def test_a_check_expires_only_after_the_limit():
    checks, future = StockChecks(), Pending()
    checks.ask(fill(), future, 0.0)
    assert checks.expired(LIMIT_S) == []
    late = checks.expired(LIMIT_S + 0.1)
    assert [c.future for c in late] == [future]
    assert checks.answered(7, future) is None  # 한도 뒤 답은 버린다


def test_a_held_fill_stays_until_taken():
    checks = StockChecks()
    checks.hold(fill())
    assert 7 in checks and checks.expired(100.0) == []
    command, future = checks.take(7)
    assert (command.goal_id, future) == (7, None) and 7 not in checks
    assert checks.take(7) is None


def test_taking_an_asked_check_gives_its_future_to_drop():
    checks, future = StockChecks(), object()
    checks.ask(fill(), future, 0.0)
    assert checks.take(7)[1] is future


def test_checks_the_core_gave_up_on_are_dropped():
    checks, future = StockChecks(), object()
    checks.ask(fill(7), future, 0.0)
    checks.hold(fill(8))
    checks.hold(fill(9))
    gone = checks.given_up(lambda goal_id: goal_id == 9)
    assert sorted(c.goal_id for c, _ in gone) == [7, 8]
    assert [f for _, f in gone if f is not None] == [future]
    assert 9 in checks and 7 not in checks and 8 not in checks
