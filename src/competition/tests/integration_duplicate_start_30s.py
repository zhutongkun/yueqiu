#!/usr/bin/env python3
"""Real-time regression check for the historical second mission execution."""

import pathlib
import sys
import threading
import time


SCRIPT_DIR = pathlib.Path(__file__).resolve().parents[1] / 'scripts' / 'navigation_transport'
sys.path.insert(0, str(SCRIPT_DIR))

from moon_mission_core import MissionLifecycle


def successful_result(index):
    return {
        'task_point_index': index,
        'status': 'success',
        'class_name_en': 'satellite',
        'class_name_cn': '\u536b\u661f',
        'confidence': 0.95,
    }


def main():
    lifecycle = MissionLifecycle()
    assert lifecycle.request_start_token('voice')
    assert lifecycle.mark_running()
    for index in (1, 2, 3):
        lifecycle.record_scene_result(index, successful_result(index))
    assert lifecycle.mark_all_tasks_finished()
    assert lifecycle.mark_returning()
    assert lifecycle.mark_returned()
    assert lifecycle.begin_announcing()
    assert lifecycle.mark_announced()
    assert lifecycle.mark_completed()

    accepted = []
    stop_at = time.monotonic() + 30.0

    def late_start_spam(source):
        while time.monotonic() < stop_at:
            accepted.append(lifecycle.request_start_token(source))
            time.sleep(0.01)

    workers = [
        threading.Thread(target=late_start_spam, args=('voice',)),
        threading.Thread(target=late_start_spam, args=('timeout',)),
        threading.Thread(target=late_start_spam, args=('service',)),
    ]
    for worker in workers:
        worker.start()
    for worker in workers:
        worker.join()

    snapshot = lifecycle.snapshot()
    assert not any(accepted)
    assert snapshot['mission_state'] == 'COMPLETED'
    assert snapshot['mission_execution_count'] == 1
    print('30-second duplicate-start regression passed:', snapshot['mission_state'])


if __name__ == '__main__':
    main()
