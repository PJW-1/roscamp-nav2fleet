"""손으로 만든 rollout 에 대한 시간초과 분류기 테스트 (Isaac 없이)."""

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
    T, K = 40, 4  # dt 0.5 s, 에피소드 20 s, env 3 은 일찍 끝남 (시간초과 아님)
    xy = np.zeros((T, K, 2))
    done = np.zeros((T, K), dtype=bool)
    done[T - 1, :] = True
    done[9, 3] = True
    rxy = np.full((T, K, 1, 2), 5.0)
    ron = np.zeros((T, K, 1), dtype=bool)
    rxy[:, 0, 0] = (0.2, 0.0)  # env 0: 0.2 m 옆에 서 있는 로봇
    ron[:, 0] = True
    rxy[:, 1, 0, 0] = 0.2 + 0.1 * np.sin(np.arange(T))  # env 1: 가까이서 움직이는 로봇
    rxy[:, 1, 0, 1] = 0.0
    ron[:, 1] = True
    z = _Z(xy=xy, done=done, dt=0.5, robot_xy=rxy, robot_on=ron,
           mover_xy=np.full((T, K, 1, 2), 5.0), mover_on=np.zeros((T, K, 1), dtype=bool))
    rows = TO.classify(z, episode_s=20.0, window_s=5.0)
    assert [(r["env"], r["kind"]) for r in rows] == [
        (0, "robot_still"), (1, "robot_moving"), (2, "alone")
    ]


def test_alone_summary_says_where_and_how_it_ended():
    T, K = 40, 4  # dt 0.5 s, 에피소드 20 s, 로봇/mover 없음: 넷 다 alone 시간초과
    xy = np.zeros((T, K, 2))
    xy[:, 0] = (1.0, 0.0)  # env 0: 노드 N1 에 정지, 앞이 트임
    xy[:, 1, 0] = 2.0 + 0.02 * (np.arange(T) % 2)  # env 1: 아무것도 없는 곳에서 2 cm 왕복
    xy[:, 2] = (0.0, 0.0)  # env 2: 노드 N0 에 정지, 0.1 m 앞에 벽
    xy[:, 3, 0] = np.arange(T) * 0.05  # env 3: 시간이 끝날 때 아직 (천천히) 주행 중
    done = np.zeros((T, K), dtype=bool)
    done[T - 1] = True
    front = np.full((T, K), 2.0)
    front[:, 2] = 0.1
    cmd = np.zeros((T, K, 2))
    z = _Z(xy=xy, done=done, dt=0.5, front_m=front, cmd=cmd,
           node_xy=np.array([[0.0, 0.0], [1.0, 0.0]]), node_names=np.array(["N0", "N1"]))
    rows = TO.classify(z, episode_s=20.0, window_s=5.0)
    s = TO.alone_summary(z, rows, n_episodes=8)
    assert s["count"] == 4 and s["per_4096"] == 2048.0
    # play.py 는 요청한 에피소드 뒤에도 기록을 이어 감: 그것은 뺌
    done2 = np.zeros((2 * T, K), dtype=bool)
    done2[[T - 1, 2 * T - 1]] = True
    z2 = _Z({**z, "xy": np.concatenate((xy, xy)), "done": done2,
             "front_m": np.concatenate((front, front)), "cmd": np.concatenate((cmd, cmd))})
    rows2 = TO.classify(z2, episode_s=20.0, window_s=5.0, max_episodes=1)
    assert len(rows2) == 4 and TO.alone_summary(z2, rows2, max_episodes=1)["tracked_episodes"] == 4
    assert s["by_motion"] == {"stopped": 1, "dithering": 1, "facing_wall": 1, "moving": 1}
    assert s["by_place"]["N0"] == 1 and s["by_place"]["N1"] == 1
