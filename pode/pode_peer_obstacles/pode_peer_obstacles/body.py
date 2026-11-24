"""다른 Pode의 몸을 덮는 원 안의 점을 계산한다(ROS 없음)."""
import math

# 몸을 덮는 원 반지름 (m): 몸 가운데에서 가장 먼 꼭짓점 + 2 mm
RADIUS_M = 0.075
# 원 가운데: 바퀴 축(base_footprint)에서 몸 가운데까지 앞쪽 거리 (m)
CENTRE_M = -0.018


def others(names: str, me: str) -> list[str]:
    """"pode1,pode2,pode3"에서 나를 뺀 이름들."""
    found = []
    for name in (part.strip() for part in names.split(",")):
        if name and name != me and name not in found:
            found.append(name)
    return found


def body_points(x: float, y: float, yaw: float, radius_m: float, step_m: float,
                centre_m: float) -> list[tuple[float, float]]:
    """자세 (x, y, yaw)의 몸을 덮는 원 안의 점들(map 좌표).

    둘레는 격자만으로 칸이 빠질 수 있어 step_m보다 촘촘히 더 찍는다.
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
