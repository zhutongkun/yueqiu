#!/usr/bin/env python3
"""Unit tests for the ROS-independent lunar competition mission core."""

import json
import os
import sys
import tempfile
import threading
import unittest
from pathlib import Path


CORE_DIRECTORY = Path(__file__).resolve().parents[1] / "scripts" / "navigation_transport"
sys.path.insert(0, str(CORE_DIRECTORY))

from moon_mission_core import (  # noqa: E402
    ANNOUNCING_RESULTS,
    COMPLETED,
    ERROR,
    RUNNING,
    STARTING,
    STOPPED,
    WAITING_FOR_START,
    MissionLifecycle,
    MultiFrameVoter,
    atomic_write_json,
)


def result(index, status="success", class_name_cn="月坑"):
    return {
        "task_point_index": index,
        "status": status,
        "class_name_cn": class_name_cn,
        "confidence": 0.9 if status == "success" else 0.0,
    }


def running_lifecycle():
    lifecycle = MissionLifecycle()
    if not lifecycle.request_start_token("voice") or not lifecycle.mark_running():
        raise AssertionError("test fixture could not enter RUNNING")
    return lifecycle


def ready_to_return_lifecycle():
    lifecycle = running_lifecycle()
    for index in (1, 2, 3):
        lifecycle.record_scene_result(index, result(index))
    lifecycle.mark_all_tasks_finished()
    return lifecycle


class FakeClock(object):
    def __init__(self):
        self.value = 0.0

    def __call__(self):
        return self.value

    def advance(self, seconds):
        self.value += seconds


class MissionLifecycleStartTests(unittest.TestCase):
    def test_voice_start_consumes_token_once(self):
        lifecycle = MissionLifecycle()
        self.assertTrue(lifecycle.request_start_token("voice"))
        self.assertFalse(lifecycle.request_start_token("timeout"))
        self.assertEqual(STARTING, lifecycle.mission_state)
        self.assertEqual(1, lifecycle.mission_execution_count)
        self.assertEqual("voice", lifecycle.start_trigger_source)

    def test_timeout_start_is_accepted_once(self):
        lifecycle = MissionLifecycle()
        self.assertTrue(lifecycle.request_start_token("timeout"))
        self.assertFalse(lifecycle.request_start_token("timeout"))
        self.assertEqual(1, lifecycle.mission_execution_count)

    def test_concurrent_voice_timer_race_has_one_winner(self):
        lifecycle = MissionLifecycle()
        barrier = threading.Barrier(16)
        accepted = []
        accepted_lock = threading.Lock()

        def request(index):
            barrier.wait()
            value = lifecycle.request_start_token("voice" if index % 2 else "timeout")
            with accepted_lock:
                accepted.append(value)

        threads = [threading.Thread(target=request, args=(index,)) for index in range(16)]
        for thread in threads:
            thread.start()
        for thread in threads:
            thread.join()

        self.assertEqual(1, accepted.count(True))
        self.assertEqual(15, accepted.count(False))
        self.assertEqual(1, lifecycle.mission_execution_count)

    def test_running_rejects_additional_start(self):
        lifecycle = running_lifecycle()
        self.assertEqual(RUNNING, lifecycle.mission_state)
        self.assertFalse(lifecycle.request_start_token("voice"))

    def test_completed_rejects_voice_and_service_start(self):
        lifecycle = ready_to_return_lifecycle()
        self.assertTrue(lifecycle.mark_returning())
        self.assertTrue(lifecycle.mark_returned())
        self.assertTrue(lifecycle.begin_announcing())
        self.assertTrue(lifecycle.mark_announced())
        self.assertTrue(lifecycle.mark_completed())
        self.assertFalse(lifecycle.request_start_token("voice"))
        self.assertFalse(lifecycle.request_start_token("service"))
        self.assertEqual(COMPLETED, lifecycle.mission_state)
        self.assertEqual(1, lifecycle.mission_execution_count)

    def test_stop_error_and_shutdown_reject_start(self):
        stopped = MissionLifecycle()
        self.assertTrue(stopped.mark_stopped())
        self.assertFalse(stopped.request_start_token("timeout"))
        self.assertEqual(STOPPED, stopped.mission_state)

        failed = MissionLifecycle()
        self.assertTrue(failed.mark_error())
        self.assertFalse(failed.request_start_token("timeout"))
        self.assertEqual(ERROR, failed.mission_state)

        shutdown = MissionLifecycle()
        self.assertFalse(shutdown.request_start_token("timeout", ros_shutdown=True))
        self.assertEqual(WAITING_FOR_START, shutdown.mission_state)

    def test_empty_trigger_source_is_rejected(self):
        lifecycle = MissionLifecycle()
        self.assertFalse(lifecycle.request_start_token(""))
        self.assertFalse(lifecycle.request_start_token(None))


class MissionLifecycleFlowTests(unittest.TestCase):
    def test_three_results_are_independent_defensive_copies(self):
        lifecycle = running_lifecycle()
        first = result(1, class_name_cn="卫星")
        lifecycle.record_scene_result(1, first)
        first["class_name_cn"] = "被外部修改"
        lifecycle.record_scene_result(2, result(2, class_name_cn="月球车"))
        lifecycle.record_scene_result(3, result(3, class_name_cn="地球"))

        snapshot = lifecycle.snapshot()
        self.assertEqual("卫星", snapshot["task_points"][0]["class_name_cn"])
        self.assertEqual("月球车", snapshot["task_points"][1]["class_name_cn"])
        self.assertEqual("地球", snapshot["task_points"][2]["class_name_cn"])
        self.assertTrue(snapshot["three_scene_recognitions_finished"])

    def test_scene_result_slot_mismatch_is_rejected(self):
        lifecycle = running_lifecycle()
        with self.assertRaises(ValueError):
            lifecycle.record_scene_result(1, result(2))

    def test_third_scene_result_does_not_request_return(self):
        lifecycle = running_lifecycle()
        for index in (1, 2, 3):
            lifecycle.record_scene_result(index, result(index))
        self.assertTrue(lifecycle.three_scene_recognitions_finished)
        self.assertFalse(lifecycle.all_competition_tasks_finished)
        self.assertFalse(lifecycle.can_begin_return())
        self.assertFalse(lifecycle.return_to_base_requested)

    def test_all_tasks_without_three_scenes_cannot_return(self):
        lifecycle = running_lifecycle()
        self.assertTrue(lifecycle.mark_all_tasks_finished())
        self.assertFalse(lifecycle.can_begin_return())
        self.assertFalse(lifecycle.mark_returning())

    def test_return_gate_requires_all_conditions_and_is_one_shot(self):
        lifecycle = ready_to_return_lifecycle()
        self.assertTrue(lifecycle.can_begin_return())
        self.assertFalse(lifecycle.can_begin_return(ros_shutdown=True))
        self.assertTrue(lifecycle.mark_returning())
        self.assertFalse(lifecycle.mark_returning())
        self.assertFalse(lifecycle.can_begin_return())

    def test_results_cannot_be_announced_before_base_arrival(self):
        lifecycle = ready_to_return_lifecycle()
        lifecycle.mark_returning()
        self.assertFalse(lifecycle.begin_announcing())
        self.assertFalse(lifecycle.mark_announced())
        self.assertFalse(lifecycle.results_announced)

    def test_result_order_and_failure_slot_are_preserved(self):
        lifecycle = running_lifecycle()
        lifecycle.record_scene_result(3, result(3, class_name_cn="火箭"))
        lifecycle.record_scene_result(1, result(1, class_name_cn="月坑"))
        lifecycle.record_scene_result(2, result(2, status="timeout", class_name_cn=""))
        task_points = lifecycle.snapshot()["task_points"]
        self.assertEqual([1, 2, 3], [item["task_point_index"] for item in task_points])
        self.assertEqual("timeout", task_points[1]["status"])

    def test_duplicate_base_and_announcement_callbacks_are_ignored(self):
        lifecycle = ready_to_return_lifecycle()
        lifecycle.mark_returning()
        self.assertTrue(lifecycle.mark_returned())
        self.assertFalse(lifecycle.mark_returned())
        self.assertTrue(lifecycle.begin_announcing())
        self.assertFalse(lifecycle.begin_announcing())
        self.assertTrue(lifecycle.mark_announced())
        self.assertFalse(lifecycle.mark_announced())
        self.assertEqual(ANNOUNCING_RESULTS, lifecycle.mission_state)

    def test_completed_state_retains_single_execution_invariant(self):
        lifecycle = ready_to_return_lifecycle()
        lifecycle.mark_returning()
        lifecycle.mark_returned()
        lifecycle.begin_announcing()
        lifecycle.mark_announced()
        lifecycle.mark_completed()
        snapshot = lifecycle.snapshot()
        self.assertTrue(snapshot["mission_started"])
        self.assertTrue(snapshot["mission_finished"])
        self.assertTrue(snapshot["start_trigger_consumed"])
        self.assertEqual(1, snapshot["mission_execution_count"])

    def test_reset_requires_terminal_state_and_no_active_thread(self):
        lifecycle = MissionLifecycle()
        self.assertFalse(lifecycle.reset())
        lifecycle.mark_stopped()
        self.assertFalse(lifecycle.reset(active_thread=True))
        self.assertTrue(lifecycle.reset(active_thread=False))
        self.assertEqual(WAITING_FOR_START, lifecycle.mission_state)
        self.assertEqual(0, lifecycle.mission_execution_count)

    def test_terminal_state_cannot_be_overwritten_by_late_callback(self):
        stopped = MissionLifecycle()
        stopped.mark_stopped()
        self.assertFalse(stopped.mark_error())
        self.assertEqual(STOPPED, stopped.mission_state)

        failed = MissionLifecycle()
        failed.mark_error()
        self.assertFalse(failed.mark_stopped())
        self.assertEqual(ERROR, failed.mission_state)


class MultiFrameVoterTests(unittest.TestCase):
    def test_votes_reach_stable_success(self):
        voter = MultiFrameVoter(vote_window=7, minimum_votes=4)
        results = [voter.add_vote(2, confidence) for confidence in (0.8, 0.9, 0.85, 0.95)]
        self.assertIsNone(results[0])
        self.assertEqual("success", results[-1]["status"])
        self.assertEqual(2, results[-1]["class_id"])
        self.assertEqual(4, results[-1]["vote_count"])
        self.assertAlmostEqual(0.875, results[-1]["confidence"])

    def test_insufficient_votes_remain_pending(self):
        voter = MultiFrameVoter(vote_window=7, minimum_votes=4)
        for _ in range(3):
            self.assertIsNone(voter.add_vote(1, 0.9))
        evaluation = voter.evaluate()
        self.assertEqual("pending", evaluation["status"])
        self.assertEqual("insufficient_votes", evaluation["reason"])

    def test_tied_vote_count_uses_average_confidence(self):
        voter = MultiFrameVoter(vote_window=6, minimum_votes=3)
        voter.add_vote(1, 0.75)
        voter.add_vote(2, 0.90)
        voter.add_vote(1, 0.80)
        self.assertIsNone(voter.add_vote(2, 0.95))
        evaluation = voter.evaluate()
        self.assertEqual("pending", evaluation["status"])
        self.assertEqual(2, evaluation["class_id"])
        self.assertEqual(2, evaluation["vote_count"])
        self.assertEqual("insufficient_votes", evaluation["reason"])

    def test_exact_tie_is_conflict_not_random_choice(self):
        voter = MultiFrameVoter(vote_window=6, minimum_votes=3, minimum_consecutive_frames=1)
        voter.add_vote(1, 0.9)
        voter.add_vote(2, 0.9)
        voter.add_vote(1, 0.9)
        self.assertIsNone(voter.add_vote(2, 0.9))
        evaluation = voter.evaluate()
        self.assertEqual("conflict", evaluation["status"])
        self.assertIsNone(evaluation["class_id"])
        self.assertEqual("tied_classes", evaluation["reason"])

    def test_consecutive_requirement_blocks_interleaved_votes(self):
        voter = MultiFrameVoter(
            vote_window=7,
            minimum_votes=3,
            minimum_consecutive_frames=2,
        )
        voter.add_vote(4, 0.9)
        voter.add_vote(4, 0.9)
        voter.add_vote(3, 0.8)
        self.assertIsNone(voter.add_vote(4, 0.9))
        self.assertEqual("insufficient_consecutive_frames", voter.evaluate()["reason"])
        successful = voter.add_vote(4, 0.9)
        self.assertEqual("success", successful["status"])

    def test_low_confidence_and_empty_frames_do_not_vote(self):
        voter = MultiFrameVoter(vote_window=7, minimum_votes=2, confidence_threshold=0.7)
        voter.add_vote(1, 0.69)
        voter.add_frame([])
        evaluation = voter.evaluate()
        self.assertEqual("pending", evaluation["status"])
        self.assertEqual(0, evaluation["accepted_detections"])
        self.assertEqual("no_detection", evaluation["reason"])

    def test_timeout_reports_without_forced_classification(self):
        clock = FakeClock()
        voter = MultiFrameVoter(
            vote_window=7,
            minimum_votes=4,
            timeout_seconds=5.0,
            clock=clock,
        )
        voter.add_vote(5, 0.9)
        clock.advance(5.1)
        evaluation = voter.evaluate()
        self.assertEqual("timeout", evaluation["status"])
        self.assertEqual(5, evaluation["class_id"])
        self.assertEqual("insufficient_votes", evaluation["reason"])
        self.assertIsNone(voter.add_vote(5, 0.9))

    def test_best_candidate_per_frame_is_selected(self):
        voter = MultiFrameVoter(vote_window=3, minimum_votes=1)
        selected = voter.add_frame(
            [
                {"class_id": 1, "confidence": 0.75},
                {"class_id": 6, "confidence": 0.92},
            ]
        )
        self.assertEqual(6, selected["class_id"])


class AtomicJsonTests(unittest.TestCase):
    def test_atomic_write_json_replaces_file_and_leaves_no_temp_file(self):
        with tempfile.TemporaryDirectory() as directory:
            destination = os.path.join(directory, "runtime", "moon_task_results.json")
            atomic_write_json(destination, {"value": 1})
            atomic_write_json(destination, {"value": 2, "中文": "月坑"})
            with open(destination, "r", encoding="utf-8") as stream:
                self.assertEqual({"value": 2, "中文": "月坑"}, json.load(stream))
            leftovers = list(Path(destination).parent.glob("*.tmp"))
            self.assertEqual([], leftovers)

    def test_lifecycle_writes_ordered_snapshot(self):
        lifecycle = running_lifecycle()
        for index in (3, 1, 2):
            lifecycle.record_scene_result(index, result(index))
        with tempfile.TemporaryDirectory() as directory:
            destination = os.path.join(directory, "result.json")
            lifecycle.atomic_write_json(destination)
            with open(destination, "r", encoding="utf-8") as stream:
                payload = json.load(stream)
        self.assertEqual([1, 2, 3], [item["task_point_index"] for item in payload["task_points"]])


if __name__ == "__main__":
    unittest.main()
