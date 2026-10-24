"""
NLP HTTP Client, DTO Models, and Offline Contract Support for Office HMI.
Complies with office-robot-nlp v1 HTTP API contracts.
"""

from __future__ import annotations

import json
import logging
import math
import os
import threading
import time
import uuid
from typing import Any, Dict, List, Literal, Optional

from pydantic import BaseModel, ConfigDict, Field
from PyQt5.QtCore import QObject, pyqtSignal
import requests
import yaml

logger = logging.getLogger(__name__)

# ==============================================================================
# 1. Pydantic Data Transfer Objects (DTOs)
# ==============================================================================

class DTO(BaseModel):
    model_config = ConfigDict(extra="ignore", populate_by_name=True)


class CreateSessionRequest(DTO):
    pass


class MessageRequest(DTO):
    messageId: str
    text: str


class SessionResponse(DTO):
    sessionId: str
    status: Literal["ACTIVE", "CLOSED", "EXPIRED"] = "ACTIVE"


class ConstraintDTO(DTO):
    ifBusy: Optional[str] = None
    ifClosed: Optional[str] = None


class TaskDTO(DTO):
    taskId: str
    sequenceIndex: int = 0
    documentText: Optional[str] = None
    documentId: Optional[str] = None
    recipientText: Optional[str] = None
    destinationText: str
    destinationId: str
    priority: str = "NORMAL"
    constraints: ConstraintDTO = Field(default_factory=ConstraintDTO)
    lifecycle: str = "READY"
    handoff: str = "DIRECT"
    documentLoad: str = "LOADED"


class PreviewDTO(DTO):
    position: int
    taskId: str


class BatchDTO(DTO):
    batchId: Optional[str] = None
    orderPolicy: Optional[str] = None
    tasks: List[TaskDTO] = Field(default_factory=list)
    orderPreview: List[PreviewDTO] = Field(default_factory=list)

    def get_ordered_tasks(self) -> List[TaskDTO]:
        """Sắp xếp tasks theo thứ tự định nghĩa trong orderPreview."""
        if not self.orderPreview:
            return self.tasks
        task_map = {t.taskId: t for t in self.tasks}
        sorted_preview = sorted(self.orderPreview, key=lambda p: p.position)
        ordered = []
        for p in sorted_preview:
            if p.taskId in task_map:
                ordered.append(task_map[p.taskId])
        # Thêm các task còn sót nếu có
        for t in self.tasks:
            if t not in ordered:
                ordered.append(t)
        return ordered


DialogueStatus = Literal[
    "TASK_READY",
    "BATCH_READY",
    "CLARIFICATION_REQUIRED",
    "DECISION_READY",
    "UNSUPPORTED",
    "DIALOGUE_ERROR",
    "SOCIAL",
    "TASK_UPDATE_ACCEPTED",
    "PENDING_CANCELLED",
]


class MessageResponse(DTO):
    sessionId: str
    messageId: str
    dialogueStatus: DialogueStatus
    reply: str
    batch: Optional[BatchDTO] = None


class RobotDTO(DTO):
    robotId: Optional[str] = None
    connection: str = "ONLINE"
    operatingState: str = "IDLE"
    locationId: Optional[str] = None
    currentTaskId: Optional[str] = None
    carryingDocumentIds: Optional[List[str]] = None
    lastUpdatedAt: Optional[str] = None
    source: str = "SIMULATION"


class PoseDTO(DTO):
    x: float
    y: float
    yaw: float = 0.0
    yawDegrees: float = 0.0


class SemanticDTO(DTO):
    mapId: str = "office_map"
    frameId: str = "map"
    doorId: str = "main_door"
    stagingPose: PoseDTO


class EventDTO(DTO):
    type: str
    source: str
    executionEvent: Optional[str] = None
    occurredAt: str


class TaskSnapshot(TaskDTO):
    sessionId: str
    batchId: Optional[str] = None
    lifecycleSource: str = "NLP"
    handoffSource: str = "NLP"
    documentLoadSource: str = "NLP"
    lastEvent: Optional[EventDTO] = None
    semantic: Optional[SemanticDTO] = None
    robotState: Optional[RobotDTO] = None


class BatchSnapshot(DTO):
    batchId: str
    sessionId: str
    orderPolicy: str
    tasks: List[TaskSnapshot] = Field(default_factory=list)
    orderPreview: List[PreviewDTO] = Field(default_factory=list)
    robotState: Optional[RobotDTO] = None


class HealthResponse(DTO):
    status: Literal["ok"] = "ok"
    service: Literal["office-robot-nlp"] = "office-robot-nlp"
    apiVersion: Literal["v1"] = "v1"


class ErrorDetail(DTO):
    code: str
    message: str


class ErrorResponse(DTO):
    error: ErrorDetail

# ==============================================================================
# 2. Offline Mock Engine (Mô phỏng Contract dựa trên config/rooms.yaml)
# ==============================================================================

def load_configured_room_anchors() -> Dict[str, Dict[str, Any]]:
    """Tải tọa độ room_anchor thực tế từ config/rooms.yaml."""
    anchors = {
        # Fallback sẵn theo đúng config/rooms.yaml thực tế của văn phòng
        "01": {"x": 6.70, "y": 2.70, "yaw": 0.0, "deg": 0.0, "name": "Phòng 1"},
        "1": {"x": 6.70, "y": 2.70, "yaw": 0.0, "deg": 0.0, "name": "Phòng 1"},
        "02": {"x": 7.45, "y": -0.60, "yaw": 0.0, "deg": 0.0, "name": "Phòng 2"},
        "2": {"x": 7.45, "y": -0.60, "yaw": 0.0, "deg": 0.0, "name": "Phòng 2"},
        "03": {"x": 14.20, "y": -0.45, "yaw": 0.0, "deg": 0.0, "name": "Phòng 3"},
        "3": {"x": 14.20, "y": -0.45, "yaw": 0.0, "deg": 0.0, "name": "Phòng 3"},
        "04": {"x": -6.45, "y": -8.80, "yaw": 0.0, "deg": 0.0, "name": "Phòng 4"},
        "4": {"x": -6.45, "y": -8.80, "yaw": 0.0, "deg": 0.0, "name": "Phòng 4"},
        "05": {"x": -11.65, "y": 4.25, "yaw": 0.0, "deg": 0.0, "name": "Phòng ăn"},
        "5": {"x": -11.65, "y": 4.25, "yaw": 0.0, "deg": 0.0, "name": "Phòng ăn"},
        "phòng ăn": {"x": -11.65, "y": 4.25, "yaw": 0.0, "deg": 0.0, "name": "Phòng ăn"},
        "ăn": {"x": -11.65, "y": 4.25, "yaw": 0.0, "deg": 0.0, "name": "Phòng ăn"},
        "06": {"x": 13.00, "y": 10.70, "yaw": 0.0, "deg": 0.0, "name": "Phòng giám đốc"},
        "6": {"x": 13.00, "y": 10.70, "yaw": 0.0, "deg": 0.0, "name": "Phòng giám đốc"},
        "phòng giám đốc": {"x": 13.00, "y": 10.70, "yaw": 0.0, "deg": 0.0, "name": "Phòng giám đốc"},
        "giám đốc": {"x": 13.00, "y": 10.70, "yaw": 0.0, "deg": 0.0, "name": "Phòng giám đốc"},
        "07": {"x": 25.10, "y": -5.00, "yaw": 0.0, "deg": 0.0, "name": "Phòng chủ tịch"},
        "7": {"x": 25.10, "y": -5.00, "yaw": 0.0, "deg": 0.0, "name": "Phòng chủ tịch"},
        "phòng chủ tịch": {"x": 25.10, "y": -5.00, "yaw": 0.0, "deg": 0.0, "name": "Phòng chủ tịch"},
        "chủ tịch": {"x": 25.10, "y": -5.00, "yaw": 0.0, "deg": 0.0, "name": "Phòng chủ tịch"},
    }

    paths = [
        "/home/duc/office_robot_ws/install/office_core/share/office_core/config/rooms.yaml",
        "/home/duc/office_robot_ws/config/rooms.yaml",
        os.path.join(os.path.dirname(__file__), "../../../config/rooms.yaml"),
        os.path.join(os.path.expanduser("~"), "office_robot_ws", "config", "rooms.yaml"),
    ]
    for cfg_path in paths:
        if os.path.exists(cfg_path):
            try:
                with open(cfg_path, "r", encoding="utf-8") as f:
                    data = yaml.safe_load(f)
                    if data and "rooms" in data:
                        for r_id, r_info in data["rooms"].items():
                            anchor = r_info.get("room_anchor") or {}
                            d_name = r_info.get("display_name", r_id)
                            ax = float(anchor.get("x", 0.0))
                            ay = float(anchor.get("y", 0.0))
                            entry = {"x": ax, "y": ay, "yaw": 0.0, "deg": 0.0, "name": d_name}
                            anchors[str(r_id)] = entry
                            if str(r_id).isdigit():
                                anchors[str(int(r_id))] = entry
                                anchors[f"{int(r_id):02d}"] = entry
                            if d_name:
                                anchors[d_name.strip().lower()] = entry
                        break
            except Exception as e:
                logger.debug(f"Không thể đọc anchor từ {cfg_path}: {e}")
    return anchors


def load_configured_room_staging() -> Dict[str, Dict[str, Any]]:
    """Tải tọa độ staging_pose thực tế ngoài cửa từ config/rooms.yaml."""
    staging = {
        # Fallback sẵn theo đúng config/rooms.yaml thực tế của văn phòng
        "01": {"x": 8.20, "y": 1.50, "yaw": 0.36397895650964407, "deg": 20.85, "name": "Phòng 1"},
        "1": {"x": 8.20, "y": 1.50, "yaw": 0.36397895650964407, "deg": 20.85, "name": "Phòng 1"},
        "02": {"x": 5.10, "y": 0.60, "yaw": -2.2367655641740063, "deg": -128.16, "name": "Phòng 2"},
        "2": {"x": 5.10, "y": 0.60, "yaw": -2.2367655641740063, "deg": -128.16, "name": "Phòng 2"},
        "03": {"x": 13.55, "y": 0.80, "yaw": -2.6191892218732646, "deg": -150.07, "name": "Phòng 3"},
        "3": {"x": 13.55, "y": 0.80, "yaw": -2.6191892218732646, "deg": -150.07, "name": "Phòng 3"},
        "04": {"x": -4.10, "y": -3.70, "yaw": -3.092128084988202, "deg": -177.17, "name": "Phòng 4"},
        "4": {"x": -4.10, "y": -3.70, "yaw": -3.092128084988202, "deg": -177.17, "name": "Phòng 4"},
        "05": {"x": -3.55, "y": 1.50, "yaw": -2.8216504154860935, "deg": -161.67, "name": "Phòng ăn"},
        "5": {"x": -3.55, "y": 1.50, "yaw": -2.8216504154860935, "deg": -161.67, "name": "Phòng ăn"},
        "phòng ăn": {"x": -3.55, "y": 1.50, "yaw": -2.8216504154860935, "deg": -161.67, "name": "Phòng ăn"},
        "ăn": {"x": -3.55, "y": 1.50, "yaw": -2.8216504154860935, "deg": -161.67, "name": "Phòng ăn"},
        "06": {"x": 11.20, "y": 5.00, "yaw": 0.6728325475937632, "deg": 38.55, "name": "Phòng giám đốc"},
        "6": {"x": 11.20, "y": 5.00, "yaw": 0.6728325475937632, "deg": 38.55, "name": "Phòng giám đốc"},
        "phòng giám đốc": {"x": 11.20, "y": 5.00, "yaw": 0.6728325475937632, "deg": 38.55, "name": "Phòng giám đốc"},
        "giám đốc": {"x": 11.20, "y": 5.00, "yaw": 0.6728325475937632, "deg": 38.55, "name": "Phòng giám đốc"},
        "07": {"x": 23.40, "y": -2.10, "yaw": -0.07485984771076686, "deg": -4.29, "name": "Phòng chủ tịch"},
        "7": {"x": 23.40, "y": -2.10, "yaw": -0.07485984771076686, "deg": -4.29, "name": "Phòng chủ tịch"},
        "phòng chủ tịch": {"x": 23.40, "y": -2.10, "yaw": -0.07485984771076686, "deg": -4.29, "name": "Phòng chủ tịch"},
        "chủ tịch": {"x": 23.40, "y": -2.10, "yaw": -0.07485984771076686, "deg": -4.29, "name": "Phòng chủ tịch"},
    }

    paths = [
        "/home/duc/office_robot_ws/install/office_core/share/office_core/config/rooms.yaml",
        "/home/duc/office_robot_ws/config/rooms.yaml",
        os.path.join(os.path.dirname(__file__), "../../../config/rooms.yaml"),
        os.path.join(os.path.expanduser("~"), "office_robot_ws", "config", "rooms.yaml"),
    ]
    for cfg_path in paths:
        if os.path.exists(cfg_path):
            try:
                with open(cfg_path, "r", encoding="utf-8") as f:
                    data = yaml.safe_load(f)
                    if data and "rooms" in data:
                        for r_id, r_info in data["rooms"].items():
                            doors = r_info.get("doors") or []
                            d_name = r_info.get("display_name", r_id)
                            if doors and len(doors) > 0:
                                sp = doors[0].get("staging_pose") or {}
                                sx = float(sp.get("x", 0.0))
                                sy = float(sp.get("y", 0.0))
                                syaw = float(sp.get("yaw", 0.0))
                                sdeg = math.degrees(syaw)
                                entry = {"x": sx, "y": sy, "yaw": syaw, "deg": sdeg, "name": d_name}
                                staging[str(r_id)] = entry
                                if str(r_id).isdigit():
                                    staging[str(int(r_id))] = entry
                                    staging[f"{int(r_id):02d}"] = entry
                                if d_name:
                                    staging[d_name.strip().lower()] = entry
                        break
            except Exception as e:
                logger.debug(f"Không thể đọc staging pose từ {cfg_path}: {e}")
    return staging


class OfflineContractMock:
    """Mô phỏng chính xác hành vi của office-robot-nlp API dựa trên cấu hình phòng thực tế (config/rooms.yaml)."""

    ROOM_STAGING = load_configured_room_staging()
    ROOM_ANCHORS = load_configured_room_anchors()
    ROOM_COORDINATES = ROOM_STAGING  # Mặc định sử dụng tọa độ Staging Pose ngoài cửa theo yêu cầu giao hàng

    @classmethod
    def to_canonical_room_id(cls, room_name_or_id: str) -> Optional[str]:
        """Chuẩn hóa tên/ID phòng về mã phòng 2 chữ số chuẩn trong config/rooms.yaml ('01' đến '07')."""
        clean = str(room_name_or_id).lower().replace("phòng", "").replace("phong", "").strip()
        if clean.isdigit():
            val = int(clean)
            cand = f"{val:02d}"
            if cand in ["01", "02", "03", "04", "05", "06", "07"]:
                return cand
        if any(w in clean for w in ["an", "ăn", "cơm", "bếp", "an uong", "ăn uống"]):
            return "05"
        if any(w in clean for w in ["giam doc", "giám đốc", "giam", "giám", "gd", "gđ"]):
            return "06"
        if any(w in clean for w in ["chu tich", "chủ tịch", "tich", "tịch", "ct"]):
            return "07"
        if clean in ["01", "02", "03", "04", "05", "06", "07"]:
            return clean
        return None

    @classmethod
    def get_room_staging(cls, room_name_or_id: str) -> Optional[Dict[str, Any]]:
        clean = str(room_name_or_id).lower().replace("phòng", "").strip()
        canon = cls.to_canonical_room_id(room_name_or_id)
        if canon and canon in cls.ROOM_STAGING:
            return cls.ROOM_STAGING[canon]
        if clean in cls.ROOM_STAGING:
            return cls.ROOM_STAGING[clean]
        if str(room_name_or_id).lower() in cls.ROOM_STAGING:
            return cls.ROOM_STAGING[str(room_name_or_id).lower()]
        return None

    @classmethod
    def get_room_anchor(cls, room_name_or_id: str) -> Optional[Dict[str, Any]]:
        clean = str(room_name_or_id).lower().replace("phòng", "").strip()
        canon = cls.to_canonical_room_id(room_name_or_id)
        if canon and canon in cls.ROOM_ANCHORS:
            return cls.ROOM_ANCHORS[canon]
        if clean in cls.ROOM_ANCHORS:
            return cls.ROOM_ANCHORS[clean]
        if str(room_name_or_id).lower() in cls.ROOM_ANCHORS:
            return cls.ROOM_ANCHORS[str(room_name_or_id).lower()]
        return None

    def __init__(self):
        self.sessions: Dict[str, Dict[str, Any]] = {}
        self.tasks_store: Dict[str, TaskSnapshot] = {}

    def health(self) -> Dict[str, Any]:
        return {"status": "ok", "service": "office-robot-nlp", "apiVersion": "v1"}

    def create_session(self) -> Dict[str, Any]:
        sid = f"mock_sess_{uuid.uuid4().hex[:8]}"
        self.sessions[sid] = {"history": [], "pending": None}
        return {"sessionId": sid, "status": "ACTIVE"}

    def _make_task(self, doc: str, room: str, prio: str = "NORMAL", idx: int = 0, sid: str = "", bid: Optional[str] = None) -> TaskDTO:
        canon_id = self.to_canonical_room_id(room) or str(room).replace("phòng", "").strip()
        coords = self.get_room_staging(canon_id) or self.get_room_staging(room) or self.get_room_anchor(canon_id) or {"x": 1.0 + idx, "y": 1.0 + idx, "yaw": 0.0, "deg": 0.0, "name": f"phòng {canon_id}"}
        disp_name = coords.get("name") or f"phòng {canon_id}"
        tid = f"task_{canon_id}_{uuid.uuid4().hex[:6]}"

        if canon_id == "05":
            dest_text = "Phòng Ăn (P05)"
        elif canon_id == "06":
            dest_text = "Phòng Giám Đốc (P06)"
        elif canon_id == "07":
            dest_text = "Phòng Chủ Tịch (P07)"
        else:
            dest_text = f"Phòng {int(canon_id) if canon_id.isdigit() else canon_id} ({disp_name})"

        task = TaskDTO(
            taskId=tid,
            sequenceIndex=idx,
            documentText=doc,
            documentId=f"doc_{doc}_{uuid.uuid4().hex[:4]}",
            recipientText=None,
            destinationText=dest_text,
            destinationId=canon_id,
            priority=prio,
            constraints=ConstraintDTO(ifBusy=None, ifClosed=None),
            lifecycle="READY",
            handoff="DIRECT",
            documentLoad="LOADED",
        )
        # Lưu snapshot với tọa độ điểm giao chính là Staging Pose ngoài cửa của phòng
        snap = TaskSnapshot(
            **task.model_dump(),
            sessionId=sid,
            batchId=bid,
            semantic=SemanticDTO(
                mapId="office_map",
                frameId="map",
                doorId="main_door",
                stagingPose=PoseDTO(
                    x=coords["x"],
                    y=coords["y"],
                    yaw=coords.get("yaw", 0.0),
                    yawDegrees=coords.get("deg", 0.0)
                ),
            ),
            robotState=RobotDTO(connection="ONLINE", operatingState="IDLE"),
        )
        self.tasks_store[tid] = snap
        return task

    def handle_message(self, sid: str, mid: str, text: str) -> Dict[str, Any]:
        import re
        if sid not in self.sessions:
            self.sessions[sid] = {"history": [], "pending": None}
        session = self.sessions[sid]
        clean_text = text.strip()
        lower_text = clean_text.lower()

        # 1. Social
        if any(w in lower_text for w in ["chào bạn", "xin chào", "hello", "hi"]):
            return MessageResponse(
                sessionId=sid,
                messageId=mid,
                dialogueStatus="SOCIAL",
                reply="Xin chào! Tôi là trợ lý Robot văn phòng. Tôi có thể hỗ trợ giao tài liệu, hợp đồng, báo cáo tới các phòng ban đã cấu hình (Phòng 1 đến 7).",
                batch=None,
            ).model_dump()

        # 2. Cancel
        if lower_text in ["hủy", "cancel", "dừng", "hủy lệnh", "huy"]:
            session["pending"] = None
            return MessageResponse(
                sessionId=sid,
                messageId=mid,
                dialogueStatus="PENDING_CANCELLED",
                reply="Đã hủy các yêu cầu nhiệm vụ đang chờ xử lý.",
                batch=None,
            ).model_dump()

        # 3. Clarification answer (Khi đang chờ làm rõ phòng chưa rõ)
        if session.get("pending") == "CLARIFICATION_REQUIRED":
            # Kiểm tra xem người dùng trả lời phòng nào (ví dụ: "Phòng 5.", "phòng 1", "giải lao", "phòng ăn")
            clarified_room = None
            for cand in ["phòng 1", "phòng 01", "phòng 2", "phòng 02", "phòng 3", "phòng 03",
                         "phòng 4", "phòng 04", "phòng 5", "phòng 05", "phòng 6", "phòng 06",
                         "phòng 7", "phòng 07", "phòng ăn", "ăn", "phòng giám đốc", "giám đốc",
                         "phòng chủ tịch", "chủ tịch"]:
                if cand in lower_text:
                    clarified_room = cand
                    break
            # Nếu người dùng chỉ gõ số, vd "5." hoặc "5"
            if not clarified_room:
                digit_match = re.search(r"\b([1-7])\b", lower_text)
                if digit_match:
                    clarified_room = f"phòng {digit_match.group(1)}"

            if clarified_room and self.to_canonical_room_id(clarified_room):
                session["pending"] = None
                bid = f"batch_{uuid.uuid4().hex[:6]}"
                t1 = self._make_task("hợp đồng", "02", "NORMAL", 0, sid, bid)
                t2 = self._make_task("báo cáo", clarified_room, "NORMAL", 1, sid, bid)
                tasks = [t1, t2]
                preview = [PreviewDTO(position=0, taskId=t1.taskId), PreviewDTO(position=1, taskId=t2.taskId)]
                batch = BatchDTO(batchId=bid, orderPolicy="PRIORITY", tasks=tasks, orderPreview=preview)
                return MessageResponse(
                    sessionId=sid,
                    messageId=mid,
                    dialogueStatus="BATCH_READY",
                    reply=f"Đã cập nhật điểm đến là {t2.destinationText}! Lộ trình giao hàng gồm Phòng 2 và {t2.destinationText} đã sẵn sàng.",
                    batch=batch,
                ).model_dump()
            else:
                return MessageResponse(
                    sessionId=sid,
                    messageId=mid,
                    dialogueStatus="CLARIFICATION_REQUIRED",
                    reply="Phòng bạn vừa nêu vẫn không nằm trong danh sách phòng đã cấu hình (Hiện có: Phòng 1 đến 7). Vui lòng chọn lại (ví dụ: 'Phòng 5.').",
                    batch=None,
                ).model_dump()

        # 4. Conflict resolution: "theo mức ưu tiên" / "giữ đúng thứ tự"
        if ("theo mức ưu tiên" in lower_text or "ưu tiên" in lower_text) and session.get("pending") == "DECISION_REQUIRED":
            session["pending"] = None
            bid = f"batch_{uuid.uuid4().hex[:6]}"
            t_high = self._make_task("hợp đồng", "02", "HIGH", 0, sid, bid)
            t_norm = self._make_task("báo cáo", "05", "NORMAL", 1, sid, bid)
            tasks = [t_norm, t_high]
            preview = [PreviewDTO(position=0, taskId=t_high.taskId), PreviewDTO(position=1, taskId=t_norm.taskId)]
            batch = BatchDTO(batchId=bid, orderPolicy="PRIORITY", tasks=tasks, orderPreview=preview)
            return MessageResponse(
                sessionId=sid,
                messageId=mid,
                dialogueStatus="BATCH_READY",
                reply="Đã xác nhận thực hiện theo mức độ ưu tiên: Giao hợp đồng (Phòng 2 - HIGH) trước, sau đó giao báo cáo (Phòng 5).",
                batch=batch,
            ).model_dump()

        if ("giữ đúng thứ tự" in lower_text or "thứ tự" in lower_text) and session.get("pending") == "DECISION_REQUIRED":
            session["pending"] = None
            bid = f"batch_{uuid.uuid4().hex[:6]}"
            t1 = self._make_task("báo cáo", "05", "NORMAL", 0, sid, bid)
            t2 = self._make_task("hợp đồng", "02", "HIGH", 1, sid, bid)
            tasks = [t1, t2]
            preview = [PreviewDTO(position=0, taskId=t1.taskId), PreviewDTO(position=1, taskId=t2.taskId)]
            batch = BatchDTO(batchId=bid, orderPolicy="STRICT_ORDER", tasks=tasks, orderPreview=preview)
            return MessageResponse(
                sessionId=sid,
                messageId=mid,
                dialogueStatus="BATCH_READY",
                reply="Đã xác nhận giữ nguyên thứ tự yêu cầu: Giao báo cáo (Phòng 5) trước, sau đó giao hợp đồng (Phòng 2).",
                batch=batch,
            ).model_dump()

        # 5. Kiểm tra phòng lạ chưa cấu hình (ví dụ phòng 8, phòng 9, phòng họp...) -> CLARIFICATION_REQUIRED
        digit_rooms = re.findall(r"(?:phòng|room|phong)\s*(\d+)", clean_text, re.IGNORECASE)
        unknown_rooms = [d for d in digit_rooms if not self.to_canonical_room_id(d)]
        named_candidates = re.findall(r"(?:phòng|room|phong)\s+([a-zA-ZÀ-ỹ0-9_ -]+?)(?:\s*(?:và|rồi|đến|nhưng|trước|,|\.|$))", clean_text, re.IGNORECASE)
        for cand in named_candidates:
            cand_clean = cand.strip()
            if not cand_clean or cand_clean.isdigit():
                continue
            if not self.to_canonical_room_id(cand_clean):
                unknown_rooms.append(cand_clean)

        if unknown_rooms:
            session["pending"] = "CLARIFICATION_REQUIRED"
            unk_desc = ", ".join([f"phòng {u}" for u in unknown_rooms])
            return MessageResponse(
                sessionId=sid,
                messageId=mid,
                dialogueStatus="CLARIFICATION_REQUIRED",
                reply=f"Hệ thống không tìm thấy '{unk_desc}' trong cấu hình bản đồ (Hiện tại có: phòng 1 đến 7). Bạn có thể nói rõ phòng nào (ví dụ: phòng 5) được không?",
                batch=None,
            ).model_dump()

        # 6. CONFLICT: Xung đột giữa thứ tự phát biểu và mức ưu tiên -> DECISION_READY
        if ("đúng thứ tự" in lower_text or "theo thứ tự" in lower_text) and "ưu tiên" in lower_text:
            session["pending"] = "DECISION_REQUIRED"
            return MessageResponse(
                sessionId=sid,
                messageId=mid,
                dialogueStatus="DECISION_READY",
                reply="Phát hiện yêu cầu có xung đột giữa thứ tự chỉ định (phòng 5 trước) và mức ưu tiên cao hơn (hợp đồng phòng 2). Vui lòng chọn: 'theo mức ưu tiên' hay 'giữ đúng thứ tự'?",
                batch=None,
            ).model_dump()

        # 7. Phân tích các kịch bản cụ thể trên các phòng đã cấu hình (P1 đến P7)

        # 7. Phân tích các kịch bản dựa trên các phòng đã cấu hình (P1 đến P7)
        # Sử dụng Regex có word-boundary \b để tránh nhầm lẫn xâu con (ví dụ: 'văn bản' chứa 'ăn')
        room_patterns = [
            ("01", re.compile(r"(?:phòng\s*0?1\b|phong\s*0?1\b|phong1\b)")),
            ("02", re.compile(r"(?:phòng\s*0?2\b|phong\s*0?2\b|phong2\b)")),
            ("03", re.compile(r"(?:phòng\s*0?3\b|phong\s*0?3\b|phong3\b)")),
            ("04", re.compile(r"(?:phòng\s*0?4\b|phong\s*0?4\b|phong4\b)")),
            ("05", re.compile(r"(?:phòng\s*0?5\b|phong\s*0?5\b|phong5\b|phòng\s*ăn\b|phong\s*an\b|\bnhà\s*ăn\b|\bđến\s*ăn\b)")),
            ("06", re.compile(r"(?:phòng\s*0?6\b|phong\s*0?6\b|phong6\b|phòng\s*giám\s*đốc\b|phong\s*giam\s*doc\b|\bgiám\s*đốc\b|\bgiam\s*doc\b)")),
            ("07", re.compile(r"(?:phòng\s*0?7\b|phong\s*0?7\b|phong7\b|phòng\s*chủ\s*tịch\b|phong\s*chu\s*tich\b|\bchủ\s*tịch\b|\bchu\s*tich\b)")),
        ]

        found_rooms = []
        for r_id, pat in room_patterns:
            m = pat.search(lower_text)
            if m:
                found_rooms.append((m.start(), r_id))

        if found_rooms:
            # Sắp xếp theo thứ tự xuất hiện trong câu
            found_rooms.sort(key=lambda x: x[0])
            ordered_rids = [r for _, r in found_rooms]

            # 7a. Kịch bản 3 phòng kèm ưu tiên hồ sơ trước (P2, P5, P7)
            if "02" in ordered_rids and "05" in ordered_rids and "07" in ordered_rids and len(ordered_rids) == 3:
                bid = f"batch_{uuid.uuid4().hex[:6]}"
                t1 = self._make_task("hợp đồng", "02", "NORMAL", 0, sid, bid)
                t2 = self._make_task("báo cáo", "05", "NORMAL", 1, sid, bid)
                t3 = self._make_task("hồ sơ", "07", "HIGH", 2, sid, bid)
                tasks = [t1, t2, t3]
                # Hồ sơ phòng 7 (Phòng chủ tịch) ưu tiên cao nhất, đưa lên vị trí 0
                preview = [
                    PreviewDTO(position=0, taskId=t3.taskId),
                    PreviewDTO(position=1, taskId=t1.taskId),
                    PreviewDTO(position=2, taskId=t2.taskId),
                ]
                batch = BatchDTO(batchId=bid, orderPolicy="PRIORITY", tasks=tasks, orderPreview=preview)
                return MessageResponse(
                    sessionId=sid,
                    messageId=mid,
                    dialogueStatus="BATCH_READY",
                    reply="Đã lập lộ trình 3 phòng! Theo mức ưu tiên, Robot sẽ giao hồ sơ đến Phòng Chủ Tịch (Phòng 7) trước, sau đó đến Phòng 2 và Phòng Ăn (Phòng 5).",
                    batch=batch,
                ).model_dump()

            # 7b. Kịch bản 2 phòng (P2 và P5)
            if "02" in ordered_rids and "05" in ordered_rids and len(ordered_rids) == 2:
                is_strict = "bắt buộc" in lower_text or "toàn bộ thứ tự" in lower_text
                bid = f"batch_{uuid.uuid4().hex[:6]}"
                prio = "HIGH" if is_strict else "NORMAL"
                policy = "STRICT_ORDER" if is_strict else "PRIORITY"
                t1 = self._make_task("hợp đồng", "02", prio, 0, sid, bid)
                t2 = self._make_task("báo cáo", "05", "NORMAL", 1, sid, bid)
                tasks = [t1, t2]
                preview = [PreviewDTO(position=0, taskId=t1.taskId), PreviewDTO(position=1, taskId=t2.taskId)]
                batch = BatchDTO(batchId=bid, orderPolicy=policy, tasks=tasks, orderPreview=preview)
                policy_desc = "giữ đúng toàn bộ thứ tự" if is_strict else "theo thứ tự tối ưu"
                return MessageResponse(
                    sessionId=sid,
                    messageId=mid,
                    dialogueStatus="BATCH_READY",
                    reply=f"Đã lập lộ trình 2 phòng ({policy_desc}): Giao hợp đồng đến Phòng 2 rồi giao báo cáo đến Phòng Ăn (Phòng 5).",
                    batch=batch,
                ).model_dump()

            # 7c. Trích xuất tài liệu / vật phẩm từ câu nói
            doc_type = "tài liệu"
            for d_kw in ["hợp đồng", "báo cáo", "hồ sơ", "văn bản", "bưu phẩm", "cà phê", "cơm trưa", "đồ ăn", "suất ăn", "vật phẩm"]:
                if d_kw in lower_text:
                    doc_type = d_kw
                    break

            # 7d. Đơn phòng (Single Room)
            if len(ordered_rids) == 1:
                single_r = ordered_rids[0]
                if single_r == "05" and doc_type == "tài liệu":
                    doc_for_single = "suất ăn / cơm trưa"
                elif single_r in ["06", "07"] and doc_type == "tài liệu":
                    doc_for_single = "hồ sơ tài liệu quan trọng"
                else:
                    doc_for_single = doc_type
                t = self._make_task(doc_for_single, single_r, "NORMAL", 0, sid, None)
                batch = BatchDTO(batchId=None, orderPolicy="PRIORITY", tasks=[t], orderPreview=[PreviewDTO(position=0, taskId=t.taskId)])
                return MessageResponse(
                    sessionId=sid,
                    messageId=mid,
                    dialogueStatus="TASK_READY",
                    reply=f"Đã nhận lệnh: Giao {doc_for_single} đến {t.destinationText}. Nhiệm vụ đã sẵn sàng thực hiện.",
                    batch=batch,
                ).model_dump()

            # 7e. Đa phòng tổng quát (Multi-room)
            bid = f"batch_{uuid.uuid4().hex[:6]}"
            is_strict = "bắt buộc" in lower_text or "thứ tự" in lower_text
            policy = "STRICT_ORDER" if is_strict else "PRIORITY"
            tasks = []
            preview = []
            for i, r_id in enumerate(ordered_rids):
                t = self._make_task(f"{doc_type} {i+1}", r_id, "NORMAL", i, sid, bid)
                tasks.append(t)
                preview.append(PreviewDTO(position=i, taskId=t.taskId))

            batch = BatchDTO(batchId=bid, orderPolicy=policy, tasks=tasks, orderPreview=preview)
            names_desc = " ➔ ".join([t.destinationText for t in tasks])
            return MessageResponse(
                sessionId=sid,
                messageId=mid,
                dialogueStatus="BATCH_READY",
                reply=f"Đã lập lộ trình giao {doc_type} tới {len(tasks)} phòng đã cấu hình: {names_desc}.",
                batch=batch,
            ).model_dump()

        # Không nhận diện được
        return MessageResponse(
            sessionId=sid,
            messageId=mid,
            dialogueStatus="UNSUPPORTED",
            reply="Tôi chưa hiểu yêu cầu này. Bạn hãy thử các lệnh giao tài liệu tới các phòng đã cấu hình (Phòng 1 đến 7), ví dụ: 'Giao tài liệu đến phòng 2' hoặc 'Giao hợp đồng đến phòng 2 và báo cáo đến phòng 5'.",
            batch=None,
        ).model_dump()

    def get_task(self, task_id: str) -> Optional[Dict[str, Any]]:
        snap = self.tasks_store.get(task_id)
        return snap.model_dump() if snap else None



# ==============================================================================
# 3. NlpHttpClient (QObject) - Quản lý Session, Health & HTTP API
# ==============================================================================

class NlpHttpClient(QObject):
    """
    HTTP Client chuẩn hóa cho office-robot-nlp v1 API.
    Hỗ trợ session lifecycle, health check, message exchange, snapshot queries,
    và tự động fallback sang OfflineContractMock khi không có server backend.
    """
    response_received = pyqtSignal(object)       # MessageResponse hoặc dict
    task_snapshot_received = pyqtSignal(object)  # TaskSnapshot hoặc dict
    session_created = pyqtSignal(str)            # session_id
    request_failed = pyqtSignal(str)             # lỗi
    request_started = pyqtSignal()
    status_updated = pyqtSignal(str)             # text status
    health_checked = pyqtSignal(bool, str)       # (is_healthy, message)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.base_url = "http://localhost:8000"
        self.session_id: Optional[str] = None
        self.is_busy = False
        self.use_mock_mode = False  # Bật nếu người dùng chọn hoặc khi server offline
        self.mock_engine = OfflineContractMock()

    def set_base_url(self, url: str):
        url = url.strip().rstrip("/")
        if url:
            self.base_url = url

    def set_mock_mode(self, enabled: bool):
        self.use_mock_mode = enabled
        if enabled:
            self.status_updated.emit("● Chế độ Giả lập Offline (Contract Mock)")
        else:
            self.status_updated.emit("● Chế độ Kết nối HTTP Trực tiếp")

    def test_health(self, endpoint_url: Optional[str] = None, timeout: float = 3.0):
        url = (endpoint_url or self.base_url).strip().rstrip("/")
        self.status_updated.emit("Đang kiểm tra kết nối /health...")
        thread = threading.Thread(
            target=self._worker_health,
            args=(url, timeout),
            daemon=True
        )
        thread.start()

    def test_connection(self, endpoint_url: Optional[str] = None, timeout: float = 3.0):
        """Tương thích ngược: gọi test_health."""
        self.test_health(endpoint_url, timeout)

    def _worker_health(self, url: str, timeout: float):
        if self.use_mock_mode:
            time.sleep(0.15)
            self.status_updated.emit("Kết nối tốt: Giả lập Offline (Mock contract v1)")
            self.health_checked.emit(True, "Mock Engine: office-robot-nlp (v1) OK")
            return

        health_url = f"{url}/health"
        try:
            resp = requests.get(health_url, timeout=timeout)
            if resp.status_code == 200:
                data = resp.json()
                srv = data.get("service", "unknown")
                ver = data.get("apiVersion", "v1")
                msg = f"Kết nối tốt: {srv} ({ver})"
                self.status_updated.emit(msg)
                self.health_checked.emit(True, msg)
            else:
                msg = f"Server phản hồi HTTP {resp.status_code}"
                self.status_updated.emit(msg)
                self.health_checked.emit(False, msg)
        except requests.exceptions.ConnectionError:
            msg = f"Không thể kết nối đến {health_url}. Server chưa chạy."
            self.status_updated.emit("Lỗi: Không kết nối được server")
            self.health_checked.emit(False, msg)
        except Exception as e:
            msg = f"Lỗi kiểm tra health: {str(e)}"
            self.status_updated.emit("Lỗi kết nối")
            self.health_checked.emit(False, msg)

    def create_new_session(self, url: Optional[str] = None):
        target_url = (url or self.base_url).strip().rstrip("/")
        self.status_updated.emit("Đang khởi tạo phiên hội thoại mới...")
        thread = threading.Thread(
            target=self._worker_create_session,
            args=(target_url,),
            daemon=True
        )
        thread.start()

    def _worker_create_session(self, url: str):
        if self.use_mock_mode:
            res = self.mock_engine.create_session()
            self.session_id = res["sessionId"]
            self.session_created.emit(self.session_id)
            self.status_updated.emit(f"Phiên mới: {self.session_id} (Mock)")
            return

        try:
            resp = requests.post(f"{url}/v1/sessions", json={}, timeout=5.0)
            resp.raise_for_status()
            data = resp.json()
            self.session_id = data.get("sessionId")
            if self.session_id:
                self.session_created.emit(self.session_id)
                self.status_updated.emit(f"Đã mở phiên: {self.session_id}")
            else:
                self.request_failed.emit("Server không trả về sessionId hợp lệ.")
        except Exception as e:
            # Tự động gợi ý chuyển Mock nếu offline
            self.status_updated.emit("Không tạo được session trên server")
            self.request_failed.emit(f"Lỗi tạo phiên trên {url}: {str(e)}")

    def send_prompt(self, endpoint_url: str, prompt: str, timeout: float = 12.0):
        if self.is_busy:
            self.request_failed.emit("Yêu cầu trước đó đang được xử lý, vui lòng đợi...")
            return
        self.is_busy = True
        self.request_started.emit()
        self.status_updated.emit("Đang gửi tin nhắn tới NLP API...")
        thread = threading.Thread(
            target=self._worker_send_message,
            args=(endpoint_url, prompt, timeout),
            daemon=True
        )
        thread.start()

    def _worker_send_message(self, endpoint_url: str, prompt: str, timeout: float):
        url = endpoint_url.strip().rstrip("/")
        # Tự động loại bỏ đuôi /api/chat nếu người dùng nhập URL cũ
        if url.endswith("/api/chat"):
            url = url[:-9]
        self.base_url = url

        try:
            # 1. Chế độ Mock
            if self.use_mock_mode:
                time.sleep(0.3)
                if not self.session_id:
                    s_res = self.mock_engine.create_session()
                    self.session_id = s_res["sessionId"]
                    self.session_created.emit(self.session_id)
                mid = f"msg_{uuid.uuid4().hex[:8]}"
                raw_data = self.mock_engine.handle_message(self.session_id, mid, prompt)
                parsed = self._parse_response(raw_data)
                self.status_updated.emit("Đã nhận phản hồi (Mock Contract)")
                self.response_received.emit(parsed)
                return

            # 2. Chế độ Real HTTP API
            # Bước A: Đảm bảo có session
            if not self.session_id:
                try:
                    s_resp = requests.post(f"{url}/v1/sessions", json={}, timeout=timeout)
                    s_resp.raise_for_status()
                    self.session_id = s_resp.json().get("sessionId")
                    if self.session_id:
                        self.session_created.emit(self.session_id)
                except Exception as se:
                    logger.warning(f"Không thể tạo session qua /v1/sessions: {se}")

            # Bước B: Gửi tin nhắn
            mid = f"msg_{uuid.uuid4().hex[:8]}"
            msg_payload = {"messageId": mid, "text": prompt}

            # Thử gửi qua endpoint chuẩn v1: /v1/sessions/{sessionId}/messages
            sent = False
            data = None
            if self.session_id:
                msg_url = f"{url}/v1/sessions/{self.session_id}/messages"
                try:
                    resp = requests.post(msg_url, json=msg_payload, timeout=timeout)
                    if resp.status_code == 200:
                        data = resp.json()
                        sent = True
                    elif resp.status_code == 404:
                        # Session có thể đã hết hạn, tạo lại
                        s_resp = requests.post(f"{url}/v1/sessions", json={}, timeout=timeout)
                        if s_resp.status_code in [200, 201]:
                            self.session_id = s_resp.json().get("sessionId")
                            self.session_created.emit(self.session_id)
                            msg_url = f"{url}/v1/sessions/{self.session_id}/messages"
                            resp2 = requests.post(msg_url, json=msg_payload, timeout=timeout)
                            if resp2.status_code == 200:
                                data = resp2.json()
                                sent = True
                except requests.exceptions.RequestException:
                    pass

            # Fallback nếu backend dùng endpoint cũ hoặc custom
            if not sent:
                for alt_path in ["/v1/messages", "/api/chat", "/messages"]:
                    try:
                        alt_url = f"{url}{alt_path}"
                        resp = requests.post(
                            alt_url,
                            json={"message": prompt, "text": prompt, "query": prompt, "messageId": mid},
                            timeout=timeout
                        )
                        if resp.status_code == 200:
                            data = resp.json()
                            sent = True
                            break
                    except Exception:
                        continue

            if not sent or data is None:
                raise ConnectionError(
                    f"Không thể kết nối đến NLP API tại {url}. Bạn có thể bật 'Chế độ Giả lập Offline' để trải nghiệm ngay."
                )

            parsed = self._parse_response(data)
            self.status_updated.emit("Đã nhận phản hồi từ NLP API")
            self.response_received.emit(parsed)

        except requests.exceptions.Timeout:
            self.status_updated.emit("Lỗi: Timeout")
            self.request_failed.emit(f"NLP API Timeout: Không có phản hồi sau {timeout}s.")
        except requests.exceptions.ConnectionError:
            self.status_updated.emit("Lỗi: Connection Refused")
            self.request_failed.emit(
                f"Không thể kết nối đến {url}.\nGợi ý: Bật 'Chế độ Giả lập Offline (Mock)' ở panel bên phải để test trực tiếp!"
            )
        except Exception as e:
            self.status_updated.emit("Lỗi")
            self.request_failed.emit(f"Lỗi gọi NLP API: {str(e)}")
        finally:
            self.is_busy = False

    def fetch_task_snapshot(self, task_id: str):
        """Lấy chi tiết TaskSnapshot (bao gồm stagingPose coordinates) từ API."""
        thread = threading.Thread(
            target=self._worker_fetch_task,
            args=(task_id,),
            daemon=True
        )
        thread.start()

    def _worker_fetch_task(self, task_id: str):
        if self.use_mock_mode:
            snap = self.mock_engine.get_task(task_id)
            if snap:
                self.task_snapshot_received.emit(snap)
            return

        try:
            url = f"{self.base_url}/v1/tasks/{task_id}"
            resp = requests.get(url, timeout=4.0)
            if resp.status_code == 200:
                self.task_snapshot_received.emit(resp.json())
        except Exception as e:
            logger.debug(f"Không thể tải TaskSnapshot {task_id}: {e}")

    def _parse_response(self, data: Any) -> Dict[str, Any]:
        """Chuẩn hóa dữ liệu theo MessageResponse và DTO contract."""
        if not isinstance(data, dict):
            return {
                "sessionId": self.session_id or "N/A",
                "messageId": "N/A",
                "dialogueStatus": "UNSUPPORTED",
                "reply": str(data),
                "batch": None,
                "raw": data
            }

        # Kiểm tra nếu là MessageResponse chuẩn Pydantic
        try:
            mr = MessageResponse.model_validate(data)
            ordered_tasks = [t.model_dump() for t in mr.batch.get_ordered_tasks()] if mr.batch else []
            return {
                "sessionId": mr.sessionId,
                "messageId": mr.messageId,
                "dialogueStatus": mr.dialogueStatus,
                "task": mr.dialogueStatus,
                "reply": mr.reply,
                "response": mr.reply,
                "batch": mr.batch.model_dump() if mr.batch else None,
                "batch_obj": mr.batch,
                "ordered_tasks": ordered_tasks,
                "raw": data
            }
        except Exception:
            pass

        # Fallback phân tích mềm nếu server trả về định dạng nhẹ
        reply = data.get("reply") or data.get("response") or data.get("message") or ""
        status = data.get("dialogueStatus") or data.get("status") or "SOCIAL"
        batch_data = data.get("batch")

        batch_obj = None
        ordered_tasks = []
        if isinstance(batch_data, dict):
            try:
                batch_obj = BatchDTO.model_validate(batch_data)
                ordered_tasks = [t.model_dump() for t in batch_obj.get_ordered_tasks()]
            except Exception:
                pass
        elif isinstance(batch_data, list):
            ordered_tasks = batch_data

        return {
            "sessionId": data.get("sessionId") or self.session_id or "N/A",
            "messageId": data.get("messageId") or "N/A",
            "dialogueStatus": status,
            "task": status,
            "reply": reply or "(Không có nội dung phản hồi)",
            "response": reply or "(Không có nội dung phản hồi)",
            "batch": batch_data if isinstance(batch_data, dict) else None,
            "batch_obj": batch_obj,
            "ordered_tasks": ordered_tasks,
            "raw": data
        }
