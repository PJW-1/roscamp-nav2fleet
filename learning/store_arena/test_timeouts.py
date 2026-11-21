"""The timeout sorter on a hand-made rollout (no Isaac)."""

import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import timeouts as TO  # noqa: E402


class _Z(dict):
    @property
    def files(self):
        return list(self)


def test_sorts_timeouts_by_what_is_near():
    T, K = 40, 4  # dt 0.5 s, 20 s episodes; env 3 ends early (not a timeout)
    xy = np.zeros((T, K, 2))
    done = np.zeros((T, K), dtype=bool)
    done[T - 1, :] = True
    done[9, 3] = True
    rxy = np.full((T, K, 1, 2), 5.0)
    ron = np.zeros((T, K, 1), dtype=bool)
    rxy[:, 0, 0] = (0.2, 0.0)  # env 0: a robot standing 0.2 m away
    ron[:, 0] = True
    rxy[:, 1, 0, 0] = 0.2 + 0.1 * np.sin(np.arange(T))  # env 1: a robot near, moving
    rxy[:, 1, 0, 1] = 0.0
    ron[:, 1] = True
    z = _Z(xy=xy, done=done, dt=0.5, robot_xy=rxy, robot_on=ron,
           mover_xy=np.full((T, K, 1, 2), 5.0), mover_on=np.zeros((T, K, 1), dtype=bool))
    rows = TO.classify(z, episode_s=20.0, window_s=5.0)
    assert [(r["env"], r["kind"]) for r in rows] == [
        (0, "robot_still"), (1, "robot_moving"), (2, "alone")
    ]
