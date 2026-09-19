"""Learning layer for env-dr3d. Depends only on `simclient` (the Phase 2 public API).

Nothing here is imported by the simulator. The simulator describes what happened;
this package defines what matters (tasks, rewards, metrics) and how to learn.
"""
from .task import NavigationTask, TaskConfig, EpisodeResult
from .policies import Policy, RandomPolicy, WaypointPolicy

__all__ = ["NavigationTask", "TaskConfig", "EpisodeResult", "Policy", "RandomPolicy", "WaypointPolicy"]
