"""Office Delivery Integration v1 - Data Models and DTOs."""

from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple


@dataclass
class NLPTaskItem:
    task_id: str
    sequence_index: int
    document_text: str
    destination_text: str
    destination_id: str
    priority: str = "NORMAL"
    lifecycle: str = "READY"
    handoff: str = "DIRECT"
    document_load: str = "LOADED"


@dataclass
class NLPResult:
    session_id: str
    message_id: str
    dialogue_status: str  # TASK_READY, BATCH_READY, CLARIFICATION_REQUIRED, UNKNOWN
    reply: str
    tasks: List[NLPTaskItem] = field(default_factory=list)


@dataclass
class DeliveryOrder:
    request_id: str
    task_id: str
    pickup: str = "pickup-a"
    delivery: str = "02"
    priority: str = "NORMAL"
    request_state: str = "DRAFT"  # DRAFT, CONFIRMED, QUEUED, PROCESSING, COMPLETED, CANCELLED
    task_state: str = "READY"
    assignment_id: Optional[str] = None
    last_update: str = ""
    idempotency_key: str = ""


@dataclass
class DeliveryExecutionState:
    pipeline_stage: str = "DRAFT"
    # Valid pipeline stages:
    # DRAFT -> CONFIRMED -> QUEUED -> ASSIGNED -> ACKED -> NAVIGATING_PICKUP ->
    # WAITING_PICKUP_QR -> NAVIGATING_DELIVERY -> WAITING_DELIVERY_QR -> COMPLETED
    assignment_id: Optional[str] = None
    boot_id: Optional[str] = None
    goal_id: Optional[str] = None
    navigation_state: str = "IDLE"  # IDLE, MOVING, REACHED_GOAL, CANCELLED, UNKNOWN, FAILED
    robot_pose: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    distance_remaining: float = -1.0
    current_leg: str = "IDLE"  # IDLE, PICKUP, DELIVERY
    qr_state: str = "IDLE"  # IDLE, WAITING_SCAN, SCANNED, VERIFIED
    last_event: str = "None"
    error_reason: str = ""


@dataclass
class SystemDashboardState:
    # Process and node statuses
    gazebo_running: bool = False
    nav2_active: bool = False
    amcl_active: bool = False
    tf_valid: bool = False
    lidar_ok: bool = False
    odom_ok: bool = False
    localization_ok: bool = False
    backend_online: bool = False
    gateway_online: bool = False
    mission_controller_online: bool = False
    navigation_manager_online: bool = False

    # Metadata & Hardware/Sim statuses
    robot_id: str = "office-robot-1"
    map_id: str = "office-map-sim-v1"
    battery_percent: float = 0.0
    battery_low: bool = True
    water_liter: float = 0.0
    water_low: bool = True
    estop_active: bool = True
    authority_mode: str = "BLOCKED"  # BLOCKED, LOCAL, DELIVERY, UNKNOWN
    delivery_authorized: bool = False
    safe_stopped: bool = False
    allow_motion: bool = False

    # Timestamp tracking for stale detection (monotonic seconds)
    last_authority_time: float = 0.0
    last_safe_stopped_time: float = 0.0
    last_robot_status_time: float = 0.0
    last_nav_status_time: float = 0.0
    last_backend_check_time: float = 0.0


# Stages in order for visual pipeline
PIPELINE_STAGES = [
    "DRAFT",
    "CONFIRMED",
    "QUEUED",
    "ASSIGNED",
    "ACKED",
    "NAVIGATING_PICKUP",
    "WAITING_PICKUP_QR",
    "NAVIGATING_DELIVERY",
    "WAITING_DELIVERY_QR",
    "COMPLETED",
]

# 12 Test A Steps
TEST_A_STEPS = [
    "1. CONNECT BACKEND",
    "2. CHECK ROS 2 / NAV2",
    "3. CHECK SAFETY",
    "4. CHECK RECONCILIATION",
    "5. CREATE NLP TASK",
    "6. CREATE DRAFT",
    "7. CONFIRM ORDER",
    "8. MONITOR PICKUP",
    "9. CONFIRM QR PICKUP",
    "10. MONITOR DELIVERY",
    "11. CONFIRM QR DELIVERY",
    "12. VERIFY COMPLETED",
]

