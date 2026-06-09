from __future__ import annotations

from dataclasses import dataclass


WORKFLOW_STAGES = (
    "profile_static",
    "zero_drift",
    "k_identification",
    "static_sequence",
    "profile_balanced",
    "training_balanced",
    "profile_fast",
    "training_fast",
    "return_zero",
    "finish",
)


@dataclass
class WorkflowState:
    active: bool = False
    paused: bool = False
    stage_index: int = -1
    stage: str = ""
    profile_retry_count: int = 0
    completed_static_points: int = 0
    invalid_static_points: int = 0
    skipped_training_targets: int = 0
    random_seed: int = 0
    failure_reason: str = ""

    def start(self, random_seed: int) -> str:
        self.active = True
        self.paused = False
        self.stage_index = 0
        self.stage = WORKFLOW_STAGES[0]
        self.profile_retry_count = 0
        self.completed_static_points = 0
        self.invalid_static_points = 0
        self.skipped_training_targets = 0
        self.random_seed = int(random_seed)
        self.failure_reason = ""
        return self.stage

    def advance(self) -> str:
        if not self.active:
            return ""
        self.stage_index += 1
        if self.stage_index >= len(WORKFLOW_STAGES):
            self.active = False
            self.stage = ""
            return ""
        self.stage = WORKFLOW_STAGES[self.stage_index]
        self.profile_retry_count = 0
        return self.stage

    def fail(self, reason: str) -> None:
        self.active = False
        self.failure_reason = str(reason)

    @property
    def progress_text(self) -> str:
        if not self.active or self.stage_index < 0:
            return "未运行"
        return f"{self.stage_index + 1}/{len(WORKFLOW_STAGES)}"
