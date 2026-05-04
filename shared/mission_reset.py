"""
RoboCupRescue RMRC 2026 - Mission Reset helper
==============================================
Operator-initiated mini-mission reset broadcast (rulebook 2.5).

Reuses the autonomy port (5560). All long-lived nodes (autonomy executor,
vision node, GUI) recognise ``msg_type == "mission_reset"`` and clear their
in-mission state without dropping engine handles or persistent best scores.

Also exposes ``BestScoreTracker`` which keeps the running best score per test
across mini-missions in ``~/.rmrc/best_scores.json``. The Best-in-Class breadth
widget reads this file.
"""
from __future__ import annotations

import json
import os
import time
import uuid
from dataclasses import dataclass
from typing import Any, Dict, Iterable, Optional

try:
    from .constants import (
        BIC_BREADTH_TARGET,
        TEST_TYPES,
        ZMQ_PORT_AUTONOMY,
    )
except ImportError:  # pragma: no cover - allow direct import from outside the package
    from shared.constants import (  # type: ignore
        BIC_BREADTH_TARGET,
        TEST_TYPES,
        ZMQ_PORT_AUTONOMY,
    )


MISSION_RESET_TYPE = "mission_reset"


def make_mission_reset_message(
    mission_id: Optional[str] = None,
    mini_mission_index: int = 0,
    reason: str = "operator",
) -> Dict[str, Any]:
    """Build the JSON-serialisable mission-reset broadcast payload."""
    return {
        "msg_type": MISSION_RESET_TYPE,
        "mission_id": mission_id or uuid.uuid4().hex[:8],
        "mini_mission_index": int(mini_mission_index),
        "reason": reason or "operator",
        "mode": "reset",  # so autonomy_executor.handle_command() accepts it
        "timestamp": time.time(),
    }


def is_mission_reset(msg: Optional[Dict[str, Any]]) -> bool:
    """True if a parsed JSON dict is a mission-reset broadcast."""
    if not isinstance(msg, dict):
        return False
    return msg.get("msg_type") == MISSION_RESET_TYPE


def broadcast_mission_reset(
    socket,
    mission_id: Optional[str] = None,
    mini_mission_index: int = 0,
    reason: str = "operator",
) -> Dict[str, Any]:
    """Send a mission-reset on the given ZMQ socket and return the payload.

    The socket is expected to already be connected to the autonomy port (5560).
    Falls through silently on non-ZMQ test sockets that expose ``send_json``.
    """
    payload = make_mission_reset_message(mission_id, mini_mission_index, reason)
    try:
        # Best effort; many GUI sockets share one autonomy PUB.
        socket.send_json(payload)
    except Exception:
        # Allow callers to log; never crash the operator path on a network blip.
        pass
    return payload


# ---------------------------------------------------------------------------
# Best-score persistence (Best-in-Class breadth)
# ---------------------------------------------------------------------------

_DEFAULT_PATH = os.path.expanduser("~/.rmrc/best_scores.json")


class BestScoreTracker:
    """Persistent per-test best scores in ``~/.rmrc/best_scores.json``.

    The file is a flat JSON object keyed by test_type (see ``TEST_TYPES``).
    Each value is the best total points the team has ever scored for that test
    (across mini-missions, sessions, and reboots). Used to compute Best-in-Class
    breadth (>=80% of available tests with positive score).
    """

    def __init__(self, path: str = _DEFAULT_PATH):
        self.path = path
        self._data: Dict[str, float] = {}
        self._load()

    def _load(self) -> None:
        if not os.path.isfile(self.path):
            return
        try:
            with open(self.path, "r", encoding="utf-8") as f:
                raw = json.load(f)
            if isinstance(raw, dict):
                self._data = {k: float(v) for k, v in raw.items() if isinstance(v, (int, float))}
        except Exception:
            self._data = {}

    def _save(self) -> None:
        try:
            os.makedirs(os.path.dirname(self.path), exist_ok=True)
            with open(self.path, "w", encoding="utf-8") as f:
                json.dump(self._data, f, indent=2, sort_keys=True)
        except Exception:
            pass

    def best(self, test_type: str) -> float:
        return float(self._data.get(test_type, 0.0))

    def record(self, test_type: str, points: float) -> bool:
        """Record a mission score. Returns True if it was a new best."""
        if not test_type or points is None:
            return False
        try:
            value = float(points)
        except (TypeError, ValueError):
            return False
        prev = self._data.get(test_type, 0.0)
        if value > prev:
            self._data[test_type] = value
            self._save()
            return True
        return False

    def all_scores(self) -> Dict[str, float]:
        return dict(self._data)

    def positive_count(self, tests: Iterable[str] = TEST_TYPES) -> int:
        return sum(1 for t in tests if self._data.get(t, 0.0) > 0)

    def breadth_ratio(self, tests: Iterable[str] = TEST_TYPES) -> float:
        tests = list(tests)
        if not tests:
            return 0.0
        return self.positive_count(tests) / float(len(tests))

    def meets_breadth_target(self, target: float = BIC_BREADTH_TARGET, tests: Iterable[str] = TEST_TYPES) -> bool:
        return self.breadth_ratio(tests) >= float(target)


# ---------------------------------------------------------------------------
# Mixin for subscribers (vision_node, autonomy_executor)
# ---------------------------------------------------------------------------

@dataclass
class MissionResetReceiverMixin:
    """Mix into long-lived ZMQ nodes so they reset in-mission state cleanly.

    Override ``handle_mission_reset(payload)`` in the subclass; default is a
    no-op. ``maybe_handle_mission_reset(msg)`` returns True if the message was
    a mission-reset and was handled.
    """

    def maybe_handle_mission_reset(self, msg: Optional[Dict[str, Any]]) -> bool:
        if not is_mission_reset(msg):
            return False
        try:
            self.handle_mission_reset(msg)  # type: ignore[attr-defined]
        except Exception:
            pass
        return True

    def handle_mission_reset(self, payload: Dict[str, Any]) -> None:  # pragma: no cover - default
        return None


__all__ = [
    "MISSION_RESET_TYPE",
    "make_mission_reset_message",
    "is_mission_reset",
    "broadcast_mission_reset",
    "BestScoreTracker",
    "MissionResetReceiverMixin",
    "ZMQ_PORT_AUTONOMY",
]
