#!/usr/bin/env python3
"""ROS-independent mission lifecycle and multi-frame voting primitives."""

import copy
import json
import os
import tempfile
import threading
import time
from collections import Counter, deque


WAITING_FOR_START = "WAITING_FOR_START"
STARTING = "STARTING"
RUNNING = "RUNNING"
RETURNING_TO_BASE = "RETURNING_TO_BASE"
ANNOUNCING_RESULTS = "ANNOUNCING_RESULTS"
COMPLETED = "COMPLETED"
STOPPED = "STOPPED"
ERROR = "ERROR"

TERMINAL_STATES = frozenset((COMPLETED, STOPPED, ERROR))


def atomic_write_json(path, payload):
    """Atomically replace *path* with a UTF-8 JSON representation of payload."""
    destination = os.path.abspath(os.fspath(path))
    directory = os.path.dirname(destination)
    if directory and not os.path.isdir(directory):
        os.makedirs(directory)

    descriptor, temporary_path = tempfile.mkstemp(
        prefix=".%s." % os.path.basename(destination),
        suffix=".tmp",
        dir=directory or None,
    )
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8", newline="\n") as stream:
            json.dump(payload, stream, ensure_ascii=False, indent=2, sort_keys=True)
            stream.write("\n")
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary_path, destination)
    except Exception:
        try:
            os.unlink(temporary_path)
        except OSError:
            pass
        raise
    return destination


class MissionLifecycle(object):
    """Thread-safe, single-run lifecycle for one competition node process."""

    def __init__(self):
        self._lock = threading.Lock()
        self._reset_unlocked()

    def _reset_unlocked(self):
        self.mission_state = WAITING_FOR_START
        self.mission_started = False
        self.mission_finished = False
        self.start_trigger_consumed = False
        self.stop_requested = False
        self.results_announced = False
        self.mission_execution_count = 0
        self.three_scene_recognitions_finished = False
        self.all_competition_tasks_finished = False
        self.return_to_base_requested = False
        self.returned_to_base = False
        self.moon_task_results = {1: None, 2: None, 3: None}
        self.start_trigger_source = None

    def request_start_token(self, trigger_source, ros_shutdown=False):
        """Consume the one allowed start token and move to STARTING."""
        source = str(trigger_source).strip() if trigger_source is not None else ""
        with self._lock:
            if (
                not source
                or ros_shutdown
                or self.mission_state != WAITING_FOR_START
                or self.mission_started
                or self.mission_finished
                or self.start_trigger_consumed
                or self.stop_requested
            ):
                return False
            self.start_trigger_consumed = True
            self.mission_started = True
            self.mission_execution_count += 1
            self.mission_state = STARTING
            self.start_trigger_source = source
            return True

    def mark_running(self):
        with self._lock:
            if self.mission_state != STARTING or self.stop_requested:
                return False
            self.mission_state = RUNNING
            return True

    def record_scene_result(self, task_point_index, result):
        """Store a defensive copy in the requested task slot."""
        if task_point_index not in (1, 2, 3):
            raise ValueError("task_point_index must be 1, 2, or 3")
        if not isinstance(result, dict):
            raise TypeError("result must be a dict")

        stored = copy.deepcopy(result)
        result_index = stored.get("task_point_index", task_point_index)
        if result_index != task_point_index:
            raise ValueError("result task_point_index does not match destination slot")
        stored["task_point_index"] = task_point_index

        with self._lock:
            self.moon_task_results[task_point_index] = stored
            self.three_scene_recognitions_finished = all(
                self.moon_task_results[index] is not None for index in (1, 2, 3)
            )
            return copy.deepcopy(stored)

    def mark_all_tasks_finished(self):
        with self._lock:
            if self.mission_state != RUNNING or self.stop_requested:
                return False
            if self.all_competition_tasks_finished:
                return False
            self.all_competition_tasks_finished = True
            return True

    def _can_begin_return_unlocked(self, ros_shutdown=False):
        return (
            not ros_shutdown
            and self.mission_state == RUNNING
            and self.three_scene_recognitions_finished
            and self.all_competition_tasks_finished
            and not self.return_to_base_requested
            and not self.returned_to_base
            and not self.stop_requested
            and not self.mission_finished
        )

    def can_begin_return(self, ros_shutdown=False):
        with self._lock:
            return self._can_begin_return_unlocked(ros_shutdown=ros_shutdown)

    def mark_returning(self, ros_shutdown=False):
        with self._lock:
            if not self._can_begin_return_unlocked(ros_shutdown=ros_shutdown):
                return False
            self.return_to_base_requested = True
            self.mission_state = RETURNING_TO_BASE
            return True

    def mark_returned(self):
        with self._lock:
            if (
                self.mission_state != RETURNING_TO_BASE
                or not self.return_to_base_requested
                or self.returned_to_base
                or self.stop_requested
            ):
                return False
            self.returned_to_base = True
            return True

    def begin_announcing(self):
        with self._lock:
            if (
                self.mission_state != RETURNING_TO_BASE
                or not self.three_scene_recognitions_finished
                or not self.all_competition_tasks_finished
                or not self.returned_to_base
                or self.results_announced
                or self.stop_requested
            ):
                return False
            self.mission_state = ANNOUNCING_RESULTS
            return True

    def mark_announced(self):
        with self._lock:
            if (
                self.mission_state != ANNOUNCING_RESULTS
                or self.results_announced
                or self.stop_requested
            ):
                return False
            self.results_announced = True
            return True

    def mark_completed(self):
        with self._lock:
            if (
                self.mission_state != ANNOUNCING_RESULTS
                or not self.mission_started
                or not self.three_scene_recognitions_finished
                or not self.all_competition_tasks_finished
                or not self.returned_to_base
                or not self.results_announced
                or self.stop_requested
            ):
                return False
            self.mission_state = COMPLETED
            self.mission_finished = True
            self.start_trigger_consumed = True
            return True

    def mark_stopped(self):
        with self._lock:
            if self.mission_state in TERMINAL_STATES:
                return False
            self.stop_requested = True
            self.start_trigger_consumed = True
            self.mission_finished = True
            self.mission_state = STOPPED
            return True

    def mark_error(self):
        with self._lock:
            if self.mission_state in TERMINAL_STATES:
                return False
            self.stop_requested = True
            self.start_trigger_consumed = True
            self.mission_finished = True
            self.mission_state = ERROR
            return True

    def reset(self, active_thread=False):
        """Reset only from a terminal state after the mission thread has exited."""
        with self._lock:
            if self.mission_state not in TERMINAL_STATES or active_thread:
                return False
            self._reset_unlocked()
            return True

    def snapshot(self):
        with self._lock:
            task_results = copy.deepcopy(self.moon_task_results)
            return {
                "mission_state": self.mission_state,
                "mission_started": self.mission_started,
                "mission_finished": self.mission_finished,
                "start_trigger_consumed": self.start_trigger_consumed,
                "stop_requested": self.stop_requested,
                "results_announced": self.results_announced,
                "mission_execution_count": self.mission_execution_count,
                "three_scene_recognitions_finished": self.three_scene_recognitions_finished,
                "all_competition_tasks_finished": self.all_competition_tasks_finished,
                "return_to_base_requested": self.return_to_base_requested,
                "returned_to_base": self.returned_to_base,
                "start_trigger_source": self.start_trigger_source,
                "moon_task_results": task_results,
                "task_points": [task_results[index] for index in (1, 2, 3)],
            }

    def atomic_write_json(self, path):
        return atomic_write_json(path, self.snapshot())


class MultiFrameVoter(object):
    """Deterministic voting over the best candidate from each image frame."""

    def __init__(
        self,
        vote_window=7,
        minimum_votes=4,
        minimum_consecutive_frames=1,
        confidence_threshold=0.70,
        timeout_seconds=15.0,
        clock=None,
    ):
        if int(vote_window) < 1:
            raise ValueError("vote_window must be positive")
        if int(minimum_votes) < 1 or int(minimum_votes) > int(vote_window):
            raise ValueError("minimum_votes must be between 1 and vote_window")
        if int(minimum_consecutive_frames) < 1:
            raise ValueError("minimum_consecutive_frames must be positive")
        if float(timeout_seconds) <= 0:
            raise ValueError("timeout_seconds must be positive")

        self.vote_window = int(vote_window)
        self.minimum_votes = int(minimum_votes)
        self.minimum_consecutive_frames = int(minimum_consecutive_frames)
        self.confidence_threshold = float(confidence_threshold)
        self.timeout_seconds = float(timeout_seconds)
        self._clock = clock or time.monotonic
        self.reset()

    def _now(self, timestamp=None):
        return float(self._clock() if timestamp is None else timestamp)

    def reset(self, now=None):
        self._frames = deque(maxlen=self.vote_window)
        self._started_at = self._now(now)
        self._frames_observed = 0
        self._accepted_detections = 0
        self._final_result = None

    def expired(self, now=None):
        return self._now(now) - self._started_at >= self.timeout_seconds

    @staticmethod
    def _normalise_candidate(candidate):
        if isinstance(candidate, dict):
            return int(candidate["class_id"]), float(candidate["confidence"])
        if isinstance(candidate, (tuple, list)) and len(candidate) >= 2:
            return int(candidate[0]), float(candidate[1])
        raise TypeError("candidate must be a dict or (class_id, confidence) pair")

    def add_vote(self, class_id, confidence, timestamp=None):
        return self.add_frame(
            [{"class_id": class_id, "confidence": confidence}],
            timestamp=timestamp,
        )

    def add_frame(self, candidates, timestamp=None):
        """Record one frame and return a result once, only when voting succeeds."""
        now = self._now(timestamp)
        if self._final_result is not None or self.expired(now):
            return None

        best = None
        for candidate in candidates or ():
            class_id, confidence = self._normalise_candidate(candidate)
            if confidence < self.confidence_threshold:
                continue
            if best is None or confidence > best[1]:
                best = (class_id, confidence)

        self._frames.append(best)
        self._frames_observed += 1
        if best is not None:
            self._accepted_detections += 1

        evaluation = self.evaluate(now=now)
        if evaluation["status"] == "success":
            self._final_result = copy.deepcopy(evaluation)
            return copy.deepcopy(evaluation)
        return None

    def _trailing_consecutive_count(self, class_id):
        count = 0
        for observation in reversed(self._frames):
            if observation is None or observation[0] != class_id:
                break
            count += 1
        return count

    def evaluate(self, now=None):
        if self._final_result is not None:
            return copy.deepcopy(self._final_result)

        confidence_by_class = {}
        for observation in self._frames:
            if observation is not None:
                confidence_by_class.setdefault(observation[0], []).append(observation[1])

        base = {
            "status": "pending",
            "class_id": None,
            "confidence": 0.0,
            "vote_count": 0,
            "detection_count": 0,
            "consecutive_count": 0,
            "score": 0.0,
            "frames_observed": self._frames_observed,
            "accepted_detections": self._accepted_detections,
            "elapsed_seconds": max(0.0, self._now(now) - self._started_at),
            "reason": "no_detection",
        }

        if confidence_by_class:
            counts = Counter(
                observation[0] for observation in self._frames if observation is not None
            )
            highest_count = max(counts.values())
            leaders = [class_id for class_id, count in counts.items() if count == highest_count]
            averages = {
                class_id: sum(confidences) / float(len(confidences))
                for class_id, confidences in confidence_by_class.items()
            }
            highest_average = max(averages[class_id] for class_id in leaders)
            leaders = [
                class_id
                for class_id in leaders
                if abs(averages[class_id] - highest_average) <= 1e-12
            ]

            if len(leaders) > 1:
                base.update(
                    {
                        "status": "conflict",
                        "vote_count": highest_count,
                        "detection_count": highest_count,
                        "confidence": highest_average,
                        "score": highest_count * highest_average,
                        "reason": "tied_classes",
                    }
                )
            else:
                winner = leaders[0]
                consecutive = self._trailing_consecutive_count(winner)
                base.update(
                    {
                        "class_id": winner,
                        "confidence": averages[winner],
                        "vote_count": counts[winner],
                        "detection_count": counts[winner],
                        "consecutive_count": consecutive,
                        "score": counts[winner] * averages[winner],
                    }
                )
                if counts[winner] < self.minimum_votes:
                    base["reason"] = "insufficient_votes"
                elif consecutive < self.minimum_consecutive_frames:
                    base["reason"] = "insufficient_consecutive_frames"
                else:
                    base["status"] = "success"
                    base["reason"] = ""

        if base["status"] != "success" and self.expired(now):
            previous_reason = base["reason"]
            base["status"] = "timeout"
            base["reason"] = previous_reason or "timeout"
        return base
