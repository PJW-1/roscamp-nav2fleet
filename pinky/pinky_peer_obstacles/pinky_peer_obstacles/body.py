"""다른 핑키의 몸 자리를 costmap에 찍을 점으로 바꾼다(ROS 없이 돈다).

라이다 빔은 다른 핑키의 몸 위로 지나가 costmap에 작은 점으로만 찍힌다. 그래서 그 로봇이 보고한
자세(/<로봇>/state)로 몸을 덮는 원을 점으로 채워 준다. 원의 가운데는 몸의 가운데다: base_footprint
(바퀴 축)에서 앞으로 centre_m(음수면 뒤)만큼이다.
"""
import math


def others(names: str, me: str) -> list[str]:
    """"pinky1,pinky2,pinky3"에서 나를 뺀 이름들(적힌 순서, 겹친 이름은 한 번)."""
    found = []
    for name in (part.strip() for part in names.split(",")):
        if name and name != me and name not in found:
            found.append(name)
    return found


def body_points(x: float, y: float, yaw: float, radius_m: float, step_m: float,
                centre_m: float) -> list[tuple[float, float]]:
    """자세 (x, y, yaw)의 로봇 몸을 덮는 원 안의 점들(map 좌표).

    안은 step_m 격자로 채우고, 가장자리는 원둘레를 step_m보다 촘촘히 돈다: 격자만으로는 둘레의
    칸이 빠질 수 있다.
    """
    cx, cy = x + centre_m * math.cos(yaw), y + centre_m * math.sin(yaw)
    n = int(radius_m / step_m)
    points = [(cx + i * step_m, cy + j * step_m)
              for i in range(-n, n + 1) for j in range(-n, n + 1)
              if math.hypot(i * step_m, j * step_m) <= radius_m]
    rim = max(8, math.ceil(2.0 * math.pi * radius_m / step_m))
    points += [(cx + radius_m * math.cos(2.0 * math.pi * k / rim),
                cy + radius_m * math.sin(2.0 * math.pi * k / rim)) for k in range(rim)]
    return points
