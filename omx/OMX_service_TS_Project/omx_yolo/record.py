"""lerobot-record 에 'yolo_opencv' 카메라 타입을 등록한 채로 진입한다.

    python -m omx_yolo.record --robot.type=omx_follower ...

왜 필요한가
────────────────────────────────────────────────────────────────────────
`lerobot-record` 는 entry point 스크립트라 `lerobot.scripts.lerobot_record.main`
만 부른다. 그 과정에서 omx_yolo 를 import 하지 않으므로 CameraConfig 레지스트리에
'yolo_opencv' 가 없고, draccus 가 다음처럼 죽는다:

    ValueError: Unknown choice 'yolo_opencv' for CameraConfig

수집(8/18)은 전부 `type: opencv` 로 했고 주석은 convert.py 가 오프라인으로
그렸기 때문에 이 경로를 한 번도 타지 않았다. 롤아웃은 실시간 주석이 필요하므로
여기서 처음 쓰인다.

인자는 그대로 통과한다. lerobot-record 의 모든 플래그를 동일하게 쓴다.
"""

from __future__ import annotations

import logging
import os
import sys
import time

import numpy as np

import omx_yolo  # noqa: F401  ← 이 import 가 'yolo_opencv' 를 등록한다
from omx_yolo.success import HOME, HOME_TOL

logger = logging.getLogger(__name__)

HOME_MOVE_S = 3.0      # 홈까지 보간 이동 시간. 급작동을 막기 위해 넉넉히 준다.
HOME_HZ = 30


def _go_home(robot) -> None:
    """리셋 구간 시작 시 팔을 홈 자세로 되돌린다.

    왜 필요한가
    ────────────────────────────────────────────────────────────────────
    lerobot_record 의 리셋 구간은 policy 도 teleop 도 없이 관측만 돈다
    (lerobot_record.py:560). 그래서 팔은 직전 에피소드가 끝난 자세에
    그대로 멈춰 있고, LeRobot 자신이 이렇게 경고한다:

        The robot won't be at its rest position at the start of the
        next episode.

    파지에 성공하면 정책이 알아서 홈으로 돌아오지만, 실패로 끝나면
    엉뚱한 자세에 남는다. 그 상태로 다음 에피소드가 시작되면

      1. success.HomeDetector 의 상태 기계가 성립하지 않아 판정이 깨진다
         ("홈에서 출발 → 떠남 → 복귀" 중 첫 단계가 없다)
      2. 정책이 학습에서 본 적 없는 초기 자세를 받아 아무 동작도 못 한다
         (2026-08-19 icecream 회차에서 실제로 발생 — 7.9초 동안 그리퍼가
          한 번도 닫히지 않았다)

    LeRobot 은 unitree_g1 에만 robot.reset() 훅을 준다. 소스를 고치지
    않기 위해 record_loop 을 감싸서, 리셋 호출(policy 도 teleop 도 없는
    호출)일 때만 홈 복귀를 먼저 수행한다.

    안전
    ────────────────────────────────────────────────────────────────────
    관절 공간에서 3초에 걸쳐 선형 보간한다. 급작동하지 않는다.
    이미 홈 근처면 아무것도 하지 않는다. 홈의 그리퍼 값은 열림(59.5)
    이므로 물고 있던 것은 놓는다.

    끄려면:  OMX_NO_AUTO_HOME=1
    """
    if os.environ.get("OMX_NO_AUTO_HOME"):
        return
    try:
        keys = list(robot.action_features)                 # "<motor>.pos" 순서
        obs = robot.get_observation()
        cur = np.array([float(obs[k]) for k in keys], dtype=np.float32)
    except Exception as e:
        logger.warning("홈 복귀 생략 — 현재 자세를 읽지 못했습니다: %s", e)
        return

    if cur.shape != HOME.shape:
        logger.warning("홈 복귀 생략 — 관절 수 불일치 (%s vs %s)", cur.shape, HOME.shape)
        return
    if np.all(np.abs(cur - HOME) <= HOME_TOL):
        logger.info("이미 홈 자세입니다 — 복귀 생략")
        return

    logger.info("리셋: 팔을 홈 자세로 되돌립니다 (%.0f초)", HOME_MOVE_S)
    n = int(HOME_MOVE_S * HOME_HZ)
    for i in range(1, n + 1):
        a = cur + (HOME - cur) * (i / n)
        robot.send_action({k: float(v) for k, v in zip(keys, a)})
        time.sleep(1.0 / HOME_HZ)


def _guard_empty_save() -> None:
    """빈 에피소드 저장 시도를 죽지 않고 넘긴다.

    → 를 에피소드 시작 직후에 누르면 프레임이 하나도 안 쌓인 채로
    save_episode() 가 불려 실행 전체가 죽는다 (2026-08-19 실제 발생):

        ValueError: You must add one or several frames with `add_frame`
                    before calling `add_episode`.

    이때까지 찍은 에피소드는 이미 디스크에 있으므로 잃지는 않지만,
    남은 회차를 이어서 못 찍고 쉘을 다시 띄워야 한다. 키 한 번에 실행이
    끊기는 것은 과하므로, 빈 버퍼면 경고만 남기고 건너뛴다.
    """
    from lerobot.datasets.lerobot_dataset import LeRobotDataset

    original = LeRobotDataset.save_episode

    def patched(self, episode_data=None, *a, **kw):
        buf = episode_data if episode_data is not None else self.episode_buffer
        if buf is not None and buf.get("size", 0) == 0:
            logger.warning("빈 에피소드입니다 — 저장하지 않고 건너뜁니다 "
                           "(→ 를 에피소드 시작 직후에 누르면 이렇게 됩니다)")
            self.clear_episode_buffer()
            return None
        return original(self, episode_data, *a, **kw)

    LeRobotDataset.save_episode = patched


def main() -> None:
    import lerobot.scripts.lerobot_record as R

    _guard_empty_save()

    original = R.record_loop

    def patched(*args, **kwargs):
        # 리셋 호출은 policy 도 teleop 도 없다. 기록 호출과 이것으로 구분한다.
        if kwargs.get("policy") is None and kwargs.get("teleop") is None:
            robot = kwargs.get("robot") or (args[0] if args else None)
            if robot is not None:
                _go_home(robot)
        return original(*args, **kwargs)

    R.record_loop = patched
    R.main()


if __name__ == "__main__":
    sys.exit(main())
