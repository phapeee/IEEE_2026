from __future__ import annotations

import json
import uuid
from dataclasses import dataclass
from typing import Any, Dict, List, Optional

from rclpy.qos import QoSDurabilityPolicy as Durability
from rclpy.qos import QoSHistoryPolicy as History
from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy as Reliability
from rmf_task_msgs.msg import ApiRequest, ApiResponse
from rmf_task_msgs.srv import CancelTask


@dataclass
class PendingRequest:
    request_id: str
    game_task_id: str
    request_kind: str
    sent_at_unix_s: float


class DispatchClient:
    def __init__(self, node, response_timeout_sec: float = 8.0):
        self._node = node
        self._response_timeout_sec = response_timeout_sec

        transient_qos = QoSProfile(
            history=History.KEEP_LAST,
            depth=10,
            reliability=Reliability.RELIABLE,
            durability=Durability.TRANSIENT_LOCAL,
        )

        self._pub = node.create_publisher(ApiRequest, 'task_api_requests', transient_qos)
        self._sub = node.create_subscription(ApiResponse, 'task_api_responses', self._on_response, transient_qos)
        self._cancel_service = node.create_client(CancelTask, 'cancel_task')

        self._pending: Dict[str, PendingRequest] = {}
        self._responses: Dict[str, Dict[str, Any]] = {}

    def send_dispatch_request(
        self,
        game_task_id: str,
        request: Dict[str, Any],
        mode: str,
        fleet: Optional[str],
        robot: Optional[str],
        now_unix_s: float,
    ) -> str:
        request_id = f'{game_task_id}.{uuid.uuid4().hex[:10]}'

        payload: Dict[str, Any]
        if mode == 'robot_targeted':
            if not fleet or not robot:
                raise ValueError('robot_targeted dispatch requires fleet and robot')
            payload = {
                'type': 'robot_task_request',
                'fleet': fleet,
                'robot': robot,
                'request': request,
            }
        else:
            payload = {
                'type': 'dispatch_task_request',
                'request': request,
            }

        msg = ApiRequest()
        msg.request_id = request_id
        msg.json_msg = json.dumps(payload)
        self._pub.publish(msg)

        self._pending[request_id] = PendingRequest(
            request_id=request_id,
            game_task_id=game_task_id,
            request_kind=payload['type'],
            sent_at_unix_s=now_unix_s,
        )

        return request_id

    def send_cancel_api_request(self, rmf_task_id: str, labels: Optional[List[str]] = None) -> str:
        request_id = f'cancel.{rmf_task_id}.{uuid.uuid4().hex[:8]}'
        payload: Dict[str, Any] = {
            'type': 'cancel_task_request',
            'task_id': rmf_task_id,
        }
        if labels:
            payload['labels'] = labels

        msg = ApiRequest()
        msg.request_id = request_id
        msg.json_msg = json.dumps(payload)
        self._pub.publish(msg)

        self._pending[request_id] = PendingRequest(
            request_id=request_id,
            game_task_id=rmf_task_id,
            request_kind='cancel_task_request',
            sent_at_unix_s=self._node.get_clock().now().nanoseconds / 1e9,
        )
        return request_id

    def send_cancel_service_request(self, rmf_task_id: str):
        if not self._cancel_service.wait_for_service(timeout_sec=0.1):
            self._node.get_logger().warn('cancel_task service unavailable')
            return None
        req = CancelTask.Request()
        req.task_id = rmf_task_id
        return self._cancel_service.call_async(req)

    def consume_response(self, request_id: str) -> Optional[Dict[str, Any]]:
        return self._responses.pop(request_id, None)

    def consume_pending_timeouts(self, now_unix_s: float) -> List[PendingRequest]:
        timed_out: List[PendingRequest] = []
        remove_ids: List[str] = []
        for request_id, pending in self._pending.items():
            if now_unix_s - pending.sent_at_unix_s > self._response_timeout_sec:
                timed_out.append(pending)
                remove_ids.append(request_id)

        for request_id in remove_ids:
            self._pending.pop(request_id, None)
            self._responses.pop(request_id, None)

        return timed_out

    def has_pending(self, request_id: str) -> bool:
        return request_id in self._pending

    def _on_response(self, msg: ApiResponse):
        pending = self._pending.pop(msg.request_id, None)
        if pending is None:
            return

        try:
            payload = json.loads(msg.json_msg) if msg.json_msg else {}
        except Exception as exc:  # pragma: no cover - defensive
            payload = {'success': False, 'errors': [{'category': 'json_parse', 'detail': str(exc)}]}

        if pending.request_kind == 'cancel_task_request':
            if not bool(payload.get('success', False)):
                self._node.get_logger().warn(
                    f'Cancel request [{msg.request_id}] rejected: {payload.get("errors", "unknown")}'
                )
            return

        payload['__response_type'] = int(msg.type)
        payload['__request_kind'] = pending.request_kind
        payload['__game_task_id'] = pending.game_task_id
        self._responses[msg.request_id] = payload
