"""받은 편지함(IR-30): 넣은 순서대로 나오고, 여러 스레드가 동시에 넣어도 빠지는 것이 없다."""
import threading

from fleet_manager.inbox import Inbox


def test_items_come_out_in_the_order_they_went_in():
    box = Inbox()
    for i in range(5):
        box.put(print, (i,))
    assert [args for _, args in box.take()] == [(0,), (1,), (2,), (3,), (4,)]
    assert box.take() == []


def test_peak_keeps_the_longest_queue():
    box = Inbox()
    for i in range(3):
        box.put(print, (i,))
    box.take()
    box.put(print, (9,))
    assert box.peak == 3 and box.qsize() == 1


def test_puts_from_many_threads_are_all_kept():
    box = Inbox()

    def fill(n):
        for i in range(500):
            box.put(print, (n, i))

    threads = [threading.Thread(target=fill, args=(n,)) for n in range(8)]
    for t in threads:
        t.start()
    for t in threads:
        t.join()
    got = box.take()
    assert len(got) == 8 * 500
    for n in range(8):
        assert [a[1] for _, a in got if a[0] == n] == list(range(500))
