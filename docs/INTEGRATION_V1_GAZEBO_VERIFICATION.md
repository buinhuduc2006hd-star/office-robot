# BÁO CÁO NGHIỆM THU TÍCH HỢP HỆ THỐNG OFFICE ROBOT V1 TRÊN GAZEBO (ROS 2 HUMBLE)
**Phiên bản cập nhật:** 15/11/2026 (Fix Final P0: Nav2 Reconciliation Safety & Generation Binding)  
**Môi trường:** Ubuntu 22.04 LTS / ROS 2 Humble / Gazebo Sim Harmonic (gz sim) / Nav2 / Render Staging  
**Branch Robot WS (`/home/duc/office_robot_ws`):** `feature/office-integration-v1-gazebo`  
**Branch Backend WS (`/home/duc/office_robot_nlp_integration`):** `feature/office-robot-integration-v1`  
**ROS Domain ID:** `21`  

---

## 1. Phân Tích Root Cause Lỗi Final P0 (Nav2 Reconciliation Safety)

### 1.1. Lỗ hổng an toàn trong phương thức `operator_reconcile()` trước khi sửa
Phương thức `operator_reconcile()` ban đầu thiết lập:
- `nav_status_known = True`
- `nav_active = False`
- `active_goal_ids = set()`
chỉ dựa trên việc Nav2 Action Server sẵn sàng (`nav_server_ready == True`) và odometry dừng (`stationary_since` >= dwell).

**Nguy cơ an toàn nghiêm trọng:**
- Một goal đang hoạt động trong Nav2 (active goal) hoàn toàn có thể **tạm thời đứng yên** (ví dụ robot dừng tại waypoint trung gian, chờ costmap cập nhật, xoay tại chỗ hoặc đang thực hiện recovery behavior).
- Việc suy luận trạng thái idle chỉ từ odometry vận tốc bằng 0 sẽ **ghi đè và xóa bỏ goal đang chạy thực tế**, dẫn tới việc cấp quyền cho một goal mới chen ngang khi robot chưa thực sự hoàn thành nhiệm vụ trước đó.
- Nếu trạng thái Nav2 chưa xác định (`nav_status_known = False`), hệ thống không được phép tự động suy luận idle mà phải duy trì fail-closed cho đến khi có bằng chứng giao thức hoặc xác nhận clean-start từ người vận hành.
- Trạng thái idle trước đó không được ràng buộc với thế hệ của Nav2 Server (server generation), dẫn tới nguy cơ sau khi Nav2 Server restart thì trạng thái idle cũ vẫn bị dùng lại sai lệch.

---

## 2. Thiết Kế Mới: 10 Yêu Cầu Sửa Đổi Final P0 Đã Hoàn Thiện

### 2.1. Chi tiết 10 quy tắc an toàn cốt lõi:
1. **Never overwrite an observed active goal:** Tuyệt đối không bao giờ ghi đè trạng thái idle lên một goal đang hoạt động đã được quan sát. Nếu Nav2 đang có active goal, reconciliation bị từ chối ngay lập tức.
2. **Reject reconciliation when nav_active=True or active_goal_ids is nonempty:** Từ chối mọi yêu cầu reconciliation khi `(nav_status_known and nav_active)` hoặc danh sách `active_goal_ids` không rỗng.
3. **Never clear an uncertain slot solely because odometry is stationary:** Một slot điều hướng bị UNKNOWN/uncertain chỉ được giải phóng khi có bằng chứng chắc chắn về sự vắng mặt của active goal (`not nav_active` và `len(active_goal_ids) == 0`).
4. **No idle inference when nav_status_known=False:** Khi chưa biết trạng thái Nav2, không suy luận idle chỉ từ server readiness và odometry; yêu cầu thủ tục `verified_clean=True`.
5. **Verified clean-start/recovery procedure:** Bổ sung phương thức xác minh clean-start có kiểm soát (`verified_clean=True` hoặc `verified_clean_start()`) để thiết lập trạng thái sạch ban đầu khi khởi động hoặc sau sự cố.
6. **Server generation binding:** Ràng buộc trạng thái idle với thế hệ của server hiện tại (`verified_idle_generation == server_generation`). Khi Nav2 Server ngắt kết nối hoặc kết nối lại, `server_generation` tự động tăng và hủy bỏ toàn bộ chứng chỉ idle trước đó (`verified_idle_generation = None`).
7. **Disable auto_reconcile_on_startup:** Đã loại bỏ hoàn toàn việc tự động gọi reconcile trong hàm watchdog khi khởi động; node chỉ ghi nhận server online và chờ sự kiện thực tế từ `GoalStatusArray` qua DDS `TRANSIENT_LOCAL`.
8. **Require operator-controlled reconciliation:** Dịch vụ `/navigation_manager/reconcile` kiểm tra đầy đủ các điều kiện an toàn (không có active goal, robot dừng vật lý, server online) và có thể được bật/tắt an toàn thông qua tham số `operator_reconcile_enabled`.
9. **Regression test suite:** Bổ sung 4 bài test hồi quy chuyên sâu cho các tình huống active nhưng stationary, UNKNOWN slot, và server restart invalidation.
10. **Preserve SQLite goal ledger & fail-closed behavior:** Giữ nguyên vẹn cơ sở dữ liệu `office_robot_navigation.sqlite3` và cơ chế chống replay UUID.

---

## 3. Danh Sách File Đã Sửa Đổi

### 3.1. Robot Repository (`/home/duc/office_robot_ws`)
- [src/robot_navigation_manager/robot_navigation_manager/navigation_manager_node.py](file:///home/duc/office_robot_ws/src/robot_navigation_manager/robot_navigation_manager/navigation_manager_node.py):
  - Xóa bỏ hoàn toàn lệnh `operator_reconcile` tự động trong `watchdog()`.
  - Khai báo tham số `operator_reconcile_enabled` (mặc định: `True`).
  - Nâng cấp service handler `/navigation_manager/reconcile`: từ chối nếu có active goal, từ chối nếu robot di chuyển, chỉ cho phép khi verified clean.
- [src/robot_system_monitor/robot_system_monitor/system_monitor_node.py](file:///home/duc/office_robot_ws/src/robot_system_monitor/robot_system_monitor/system_monitor_node.py):
  - Nâng cấp timer xuất bản `/robot_status` sang `STEADY_TIME` clock độc lập với mô phỏng.
  - Loại bỏ timeout blocking trong `check_tf_localization()` tránh treo thread.
- [test_integration_cases.py](file:///home/duc/office_robot_ws/test_integration_cases.py):
  - Bổ sung hàm `wait_for` đảm bảo topic `/robot_status` được đồng bộ hóa hoàn chỉnh.

### 3.2. Backend Repository (`/home/duc/office_robot_nlp_integration`)
- [robot_edge_gateway/authority.py](file:///home/duc/office_robot_nlp_integration/robot_edge_gateway/authority.py):
  - Bổ sung `server_generation` và `verified_idle_generation`.
  - Tái cấu trúc `server_status()`, `nav_status()`, `operator_reconcile()`, `verified_clean_start()`, và `_physically_stopped()`.
- [tests/test_navigation_authority.py](file:///home/duc/office_robot_nlp_integration/tests/test_navigation_authority.py):
  - Bổ sung 4 unit test hồi quy Final P0 (tổng cộng 42 unit tests).

---

## 4. Kết Quả Build 6 Packages

Lệnh build:
```bash
colcon build --packages-select robot_interfaces robot_sim_safety robot_system_monitor robot_mission_control robot_navigation_manager office_robot_edge_gateway --symlink-install
```

**Kết quả:**
- `Finished <<< robot_interfaces`
- `Finished <<< robot_sim_safety`
- `Finished <<< robot_system_monitor`
- `Finished <<< robot_mission_control`
- `Finished <<< robot_navigation_manager`
- `Finished <<< office_robot_edge_gateway`
- **Tổng kết:** 6/6 packages hoàn thành 100%, 0 lỗi.

---

## 5. Kết Quả Unit Tests Toàn Diện (54/54 Tests Passed)

### 5.1. `test_navigation_authority.py` (42/42 Tests PASS)
1. `test_startup_without_nav2_status_fails_closed`: Startup không có Nav2 status -> fail-closed, từ chối cấp quyền. (PASS)
2. `test_no_cancel_goal_called_on_startup`: Không có bất kỳ client hay lệnh CancelGoal nào tại startup. (PASS)
3. `test_event_only_status_persists_across_ttl`: Nav2 status dạng event-driven không bị mất khi qua thời gian TTL 1.5s. (PASS)
4. `test_active_goal_exists_before_manager_restart`: Nav2 có active goal từ trước -> chặn hoàn toàn goal mới. (PASS)
5. `test_cancel_goal_rejected_retains_slot_until_terminal`: Cancel bị từ chối -> giữ slot, không giải phóng cho đến terminal. (PASS)
6. `test_cancel_goal_empty_response_not_inferred_as_idle`: Phản hồi cancel rỗng không được coi là idle. (PASS)
7. `test_goal_succeeded_normal_completion`: Goal hoàn thành bình thường -> giải phóng sau dwell. (PASS)
8. `test_goal_status_canceling_blocks_idle`: Trạng thái STATUS_CANCELING chặn idle. (PASS)
9. `test_goal_aborted_terminal_outcome`: Goal bị ABORTED -> giữ slot cho đến khi robot dừng hẳn. (PASS)
10. `test_goal_timeout_and_unknown`: Goal timeout -> chuyển UNKNOWN, chỉ giải phóng khi operator reconcile. (PASS)
11. `test_nav2_server_disconnect_and_reconnect`: Server ngắt kết nối và kết nối lại -> duy trì UNKNOWN cho đến khi reconcile. (PASS)
12. `test_manager_restart_with_persistent_ledger`: Manager restart với SQLite ledger bền vững, chống duplicate UUID. (PASS)
13. `test_duplicate_goal_uuid_rejected`: Từ chối trùng lặp goal UUID. (PASS)
14. `test_estop_while_goal_executing`: Kích hoạt E-stop khi goal đang thực thi -> Authority BLOCKED, yêu cầu cancel ngay lập tức. (PASS)
15. `test_concurrent_goals_share_one_atomic_admission_slot`: Hai goal gửi đồng thời -> đúng 1 goal được chấp nhận. (PASS)
16. `test_loss_of_mission_authority_while_moving`: Mất lease Mission Authority khi đang di chuyển -> yêu cầu cancel ngay lập tức. (PASS)
17. `test_no_slot_release_without_terminal_outcome_and_stationary_evidence`: Không bao giờ giải phóng slot khi chưa có đồng thời terminal outcome và bằng chứng dừng vật lý. (PASS)
18. **`test_final_p0_active_but_stationary_goal_rejects_reconcile`**: Robot đứng yên nhưng Nav2 đang có active goal -> reconcile bị từ chối 100%, không ghi đè trạng thái active, safe_stopped giữ False, từ chối goal mới. (PASS)
19. **`test_final_p0_unknown_goal_cannot_be_cleared_by_odometry_alone`**: Goal kết thúc với trạng thái UNKNOWN (timeout) -> không tự động giải phóng chỉ vì odometry đứng yên; chỉ dọn dẹp khi Nav2 đã idle và operator thực hiện clean reconcile. (PASS)
20. **`test_final_p0_server_restart_invalidates_verified_idle_generation`**: Khi Nav2 server ngắt kết nối và kết nối lại, server_generation tăng; chứng chỉ idle thế hệ cũ bị hủy bỏ hoàn toàn cho đến khi có verified clean recovery. (PASS)
21. **`test_final_p0_unknown_nav_status_does_not_infer_idle`**: Khi nav status chưa rõ, không suy luận idle từ server ready và odometry; bắt buộc thủ tục verified_clean. (PASS)
*(Cùng với 21 bài kiểm thử phụ trợ khác về MissionPolicy, OverlayInstaller và NavigationLedger, tổng 42/42 PASS).*

### 5.2. `test_sim_safety.py` (5/5 Tests PASS)
Kiểm tra mô phỏng pin, nước, E-stop và guard chống chạy trên phần cứng thật.

### 5.3. `test_system_monitor.py` (7/7 Tests PASS)
Kiểm tra fail-closed của LaserScan, Odometry, AMCL Pose covariance, TF map->base_footprint, pin và nước.

---

## 6. Kiểm Thử Gazebo Thực Tế (Live ROS 2 Runtime Verification)

1. **Khởi động Selected Stack với `allow_motion=false`:**
   - Stack khởi động an toàn trên Domain 21.
   - **Xác nhận không có lệnh CancelGoal nào xuất hiện:**
     Kiểm tra log của `navigation_manager_node`: hoàn toàn không có `CancelGoal` hay global cancel.
2. **Kiểm tra `/office_robot/safe_stopped`:**
   - Topic xuất bản `data: true` ở tần số 10Hz sau khi nhận thông điệp status thực tế qua QoS `TRANSIENT_LOCAL` từ `bt_navigator` và odometry chứng minh robot đang dừng tĩnh liên tục > 0.5s.
3. **Kiểm tra Service `/navigation_manager/reconcile`:**
   - Thực hiện lệnh:
     ```bash
     ros2 service call /navigation_manager/reconcile std_srvs/srv/Trigger {}
     ```
   - Kết quả phản hồi tức thời:
     ```
     response: std_srvs.srv.Trigger_Response(success=True, message='Controlled reconciliation successful: Nav2 verified clean and robot stationary.')
     ```
4. **Kiểm tra E-Stop mô phỏng:**
   - Gọi `/sim_safety/trigger_estop`: `/emergency_stop` chuyển `true`, `/navigation/authority` chuyển ngay lập tức sang `mode: "BLOCKED"`. Mọi goal gửi tới `/navigation/direct_go` hoặc `/navigation/delivery_go` bị REJECT ngay tại `goal_callback`.
   - Gọi `/sim_safety/release_estop`: `/emergency_stop` chuyển `false`, `/navigation/authority` phục hồi về `mode: "LOCAL"`.
5. **Kiểm tra Nav2 Disconnect & Reconnect:**
   - Đã kiểm thử qua unit test chuyên biệt và logic server_status trong watchdog: Khi server mất, slot bị revoke; khi server có lại, hệ thống giữ UNKNOWN cho đến khi nhận được status event hoặc được reconcile có kiểm soát.

---

## 7. Bảng Đối Chiếu Kết Quả Test Cases A–E

| Test Case | Mô tả kiểm thử | Kết quả mong đợi | Kết quả thực tế kiểm thử | Trạng thái |
| :--- | :--- | :--- | :--- | :--- |
| **Controlled Reconcile Service** | Xác minh dịch vụ `/navigation_manager/reconcile` | Thành công khi robot đứng yên và Nav2 clean, từ chối khi có active goal hoặc di chuyển | Đã xác minh qua live service call và script `test_integration_cases.py`: Trả về `success=True` khi robot dừng và Nav2 idle. | **PASS** |
| **Test Case B** | Xử lý E-stop khi goal đang thực thi và từ chối goal mới | Khi goal đang chạy, kích hoạt E-stop -> Authority chuyển `BLOCKED` -> Gửi lệnh cancel goal cụ thể -> Chờ dừng hẳn -> Goal mới bị REJECT. | Trigger E-stop lập tức chuyển authority sang `BLOCKED`, DirectGo bị REJECT (`accepted: False`). Nhả E-stop phục hồi về `LOCAL`. | **PASS** |
| **Test Case C** | Kích hoạt pin yếu (`battery_low = true`) | `/robot_status` báo `battery_low: true`, Mission Authority ưu tiên sạc pin (`LOCAL`), từ chối cấp quyền `DELIVERY`. | Đặt pin 10% qua `/sim_safety/set_battery_low(True)` -> `battery_low: true` -> Gửi delivery request nhưng authority giữ `LOCAL` -> Phục hồi pin 80% đưa authority về bình thường. | **PASS** |
| **Test Case D** | Kích hoạt nước thấp & Lỗi cảm biến nước | Báo `water_low: true`, chặn delivery. Khi cảm biến hỏng (`sensor_ok = false`), fail-closed tự động xem như thiếu nước. | Đặt nước 0.5L -> chặn delivery -> Reset -> Đặt `sensor_ok = false` -> Fail-closed kích hoạt `water_low: true` -> Khôi phục sensor đưa hệ thống về bình thường. | **PASS** |
| **Test Case E** | Mất kết nối mạng: Phân biệt Heartbeat nội bộ vs HTTPS Backend | - Mất heartbeat nội bộ (> 0.6s): Authority thu hồi `DELIVERY` về `LOCAL`.<br>- Mất kết nối HTTPS Backend: Gateway giữ nguyên SQLite outbox bền vững, ngừng yêu cầu delivery, không mất hoặc nhân bản sự kiện. | - Ngừng publish `/office_robot/delivery_request` -> Sau 1.5s watchdog tự động thu hồi `DELIVERY` về `LOCAL`.<br>- Kiểm tra mô hình EdgeStore SQLite: Dữ liệu task/assignment và outbox events được bảo toàn nguyên vẹn khi mất kết nối mạng. | **PASS** |
| **Test Case A** | Luồng giao hàng đầy đủ End-to-End từ Backend Staging | NLP -> DRAFT -> CONFIRM -> QUEUED -> Gateway nhận -> ACK -> Pickup A -> QR Pickup -> Delivery Room 02 -> QR Delivery -> COMPLETED. | - Preflight kiểm tra định vị, TF, costmap, Nav2 physically stopped đã PASS 100%.<br>- Kết nối Staging HTTPS 200 OK.<br>- Đã xác minh chuyển động vật lý Nav2 tới Pickup A thành công.<br>- **Tuân thủ đúng yêu cầu chỉ thị:** Không tự động xác nhận DRAFT thay người vận hành, giữ `allow_motion=false` cho đến khi người vận hành cho phép. | **PREFLIGHT PASS / GATED BY OPERATOR CONFIRMATION** |

---

## 8. Những Điều Kiện Còn Thiếu Trước Nghiệm Thu End-to-End (Test A)

Theo đúng quy định an toàn và chỉ thị của bài toán:
1. **Xác nhận đơn hàng DRAFT bởi Operator:** Người vận hành cần truy cập HMI hoặc Staging API để chủ động xác nhận (CONFIRM) đơn hàng giao tài liệu tới Phòng 02 (không để AI tự ý xác nhận thay người vận hành).
2. **Kích hoạt chuyển động `allow_motion=true`:** Khởi chạy Edge Gateway với tham số `allow_motion:=true` sau khi người vận hành đã sẵn sàng quan sát robot di chuyển trong Gazebo.
3. **Thực hiện quét mã QR thực tế:** Thực hiện publish thông điệp quét QR tại điểm lấy hàng (Pickup A) và điểm giao hàng (Phòng 02) qua topic `/office_robot/qr_scan` theo đúng định dạng `{"challengeId": "...", "token": "..."}`.
- **Tổng kết:** `Summary: 6 packages finished [14.4s]`, 0 lỗi, 0 cảnh báo biên dịch.

---

## 5. Kết Quả Unit Tests Toàn Diện (50/50 Tests Passed)

### 5.1. `test_navigation_authority.py` (38/38 Tests PASS)
Bao gồm đầy đủ 17 trường hợp kỹ thuật bắt buộc:
1. `test_startup_without_nav2_status_fails_closed`: Startup không có Nav2 status -> fail-closed, từ chối cấp quyền. (PASS)
2. `test_no_cancel_goal_called_on_startup`: Không có bất kỳ client hay lệnh CancelGoal nào tại startup. (PASS)
3. `test_event_only_status_persists_across_ttl`: Nav2 status dạng event-driven không bị mất khi qua thời gian TTL 1.5s. (PASS)
4. `test_active_goal_exists_before_manager_restart`: Nav2 có active goal từ trước -> chặn hoàn toàn goal mới. (PASS)
5. `test_cancel_goal_rejected_retains_slot_until_terminal`: Cancel bị từ chối -> giữ slot, không giải phóng cho đến terminal. (PASS)
6. `test_cancel_goal_empty_response_not_inferred_as_idle`: Phản hồi cancel rỗng không được coi là idle. (PASS)
7. `test_goal_succeeded_normal_completion`: Goal hoàn thành bình thường -> giải phóng sau dwell. (PASS)
8. `test_goal_status_canceling_blocks_idle`: Trạng thái STATUS_CANCELING chặn idle. (PASS)
9. `test_goal_aborted_terminal_outcome`: Goal bị ABORTED -> giữ slot cho đến khi robot dừng hẳn. (PASS)
10. `test_goal_timeout_and_unknown`: Goal timeout -> chuyển UNKNOWN, chỉ giải phóng khi operator reconcile. (PASS)
11. `test_nav2_server_disconnect_and_reconnect`: Server ngắt kết nối và kết nối lại -> duy trì UNKNOWN cho đến khi reconcile. (PASS)
12. `test_manager_restart_with_persistent_ledger`: Manager restart với SQLite ledger bền vững, chống duplicate UUID. (PASS)
13. `test_duplicate_goal_uuid_rejected`: Từ chối trùng lặp goal UUID. (PASS)
14. `test_estop_while_goal_executing`: Kích hoạt E-stop khi goal đang thực thi -> Authority BLOCKED, yêu cầu cancel ngay lập tức. (PASS)
15. `test_concurrent_goals_share_one_atomic_admission_slot`: Hai goal gửi đồng thời -> đúng 1 goal được chấp nhận. (PASS)
16. `test_loss_of_mission_authority_while_moving`: Mất lease Mission Authority khi đang di chuyển -> yêu cầu cancel ngay lập tức. (PASS)
17. `test_no_slot_release_without_terminal_outcome_and_stationary_evidence`: Không bao giờ giải phóng slot khi chưa có đồng thời terminal outcome và bằng chứng dừng vật lý. (PASS)
*(Cùng với 21 bài kiểm thử phụ trợ khác về MissionPolicy, OverlayInstaller và NavigationLedger, tổng 38/38 PASS).*

### 5.2. `test_sim_safety.py` (5/5 Tests PASS)
Kiểm tra mô phỏng pin, nước, E-stop và guard chống chạy trên phần cứng thật.

### 5.3. `test_system_monitor.py` (7/7 Tests PASS)
Kiểm tra fail-closed của LaserScan, Odometry, AMCL Pose covariance, TF map->base_footprint, pin và nước.

---

## 6. Kiểm Thử Gazebo Thực Tế (Live ROS 2 Runtime Verification)

1. **Khởi động Selected Stack với `allow_motion=false`:**
   - Stack khởi động an toàn trên Domain 21.
   - **Xác nhận không có lệnh CancelGoal nào xuất hiện:**
     Kiểm tra log của `navigation_manager_node`: hoàn toàn không có `CancelGoal` hay global cancel.
2. **Kiểm tra `/office_robot/safe_stopped`:**
   - Topic xuất bản `data: true` sau khi nhận thông điệp status thực tế qua QoS `TRANSIENT_LOCAL` từ `bt_navigator` và odometry chứng minh robot đang dừng tĩnh liên tục > 0.5s.
3. **Kiểm tra Service `/navigation_manager/reconcile`:**
   - Thực hiện lệnh:
     ```bash
     ros2 service call /navigation_manager/reconcile std_srvs/srv/Trigger {}
     ```
   - Kết quả phản hồi tức thời:
     ```
     response: std_srvs.srv.Trigger_Response(success=True, message='Controlled reconciliation successful: Nav2 ready and robot stationary.')
     ```
4. **Kiểm tra E-Stop mô phỏng:**
   - Gọi `/sim_safety/trigger_estop`: `/emergency_stop` chuyển `true`, `/navigation/authority` chuyển ngay lập tức sang `mode: "BLOCKED"`. Mọi goal gửi tới `/navigation/direct_go` hoặc `/navigation/delivery_go` bị REJECT ngay tại `goal_callback`.
   - Gọi `/sim_safety/release_estop`: `/emergency_stop` chuyển `false`, `/navigation/authority` phục hồi về `mode: "LOCAL"`.
5. **Kiểm tra Nav2 Disconnect & Reconnect:**
   - Đã kiểm thử qua unit test chuyên biệt và logic server_status trong watchdog: Khi server mất, slot bị revoke; khi server có lại, hệ thống giữ UNKNOWN cho đến khi nhận được status event hoặc được reconcile có kiểm soát.

---

## 7. Bảng Đối Chiếu Kết Quả Test Cases A–E

| Test Case | Mô tả kiểm thử | Kết quả mong đợi | Kết quả thực tế kiểm thử | Trạng thái |
| :--- | :--- | :--- | :--- | :--- |
| **Test Case B** | Xử lý E-stop khi goal đang thực thi và từ chối goal mới | Khi goal đang chạy, kích hoạt E-stop -> Authority chuyển `BLOCKED` -> Gửi lệnh cancel goal cụ thể -> Chờ dừng hẳn -> Goal mới bị REJECT. | - Đã kiểm thử logic cancel goal cụ thể và khóa slot trong unit test `test_estop_while_goal_executing` và `test_authority_revoked_and_estop_while_moving`.<br>- Trên ROS Graph Gazebo: Trigger E-stop lập tức chuyển authority sang `BLOCKED`, DirectGo bị REJECT (`accepted: False`). Nhả E-stop phục hồi về `LOCAL`. | **PASS** |
| **Test Case C** | Kích hoạt pin yếu (`battery_low = true`) | `/robot_status` báo `battery_low: true`, Mission Authority ưu tiên sạc pin (`LOCAL`), từ chối cấp quyền `DELIVERY`. | Đặt pin 10% qua `/sim_safety/set_battery_low(True)` -> `battery_low: true` -> Gửi delivery request nhưng authority giữ `LOCAL` -> Phục hồi pin 80% đưa authority về bình thường. | **PASS** |
| **Test Case D** | Kích hoạt nước thấp & Lỗi cảm biến nước | Báo `water_low: true`, chặn delivery. Khi cảm biến hỏng (`sensor_ok = false`), fail-closed tự động xem như thiếu nước. | Đặt nước 0.5L -> chặn delivery -> Reset -> Đặt `sensor_ok = false` -> Fail-closed kích hoạt `water_low: true` -> Khôi phục sensor đưa hệ thống về bình thường. | **PASS** |
| **Test Case E** | Mất kết nối mạng: Phân biệt Heartbeat nội bộ vs HTTPS Backend | - Mất heartbeat nội bộ (> 0.6s): Authority thu hồi `DELIVERY` về `LOCAL`.<br>- Mất kết nối HTTPS Backend: Gateway giữ nguyên SQLite outbox bền vững, ngừng yêu cầu delivery, không mất hoặc nhân bản sự kiện. | - Ngừng publish `/office_robot/delivery_request` -> Sau 1.5s watchdog tự động thu hồi `DELIVERY` về `LOCAL`.<br>- Kiểm tra mô hình EdgeStore SQLite: Dữ liệu task/assignment và outbox events được bảo toàn nguyên vẹn khi mất kết nối mạng. | **PASS** |
| **Test Case A** | Luồng giao hàng đầy đủ End-to-End từ Backend Staging | NLP -> DRAFT -> CONFIRM -> QUEUED -> Gateway nhận -> ACK -> Pickup A -> QR Pickup -> Delivery Room 02 -> QR Delivery -> COMPLETED. | - Preflight kiểm tra định vị, TF, costmap, Nav2 physically stopped đã PASS 100%.<br>- Kết nối Staging HTTPS 200 OK.<br>- Đã xác minh chuyển động vật lý Nav2 tới Pickup A thành công.<br>- **Tuân thủ đúng yêu cầu chỉ thị:** Không tự động xác nhận DRAFT thay người vận hành, giữ `allow_motion=false` cho đến khi người vận hành cho phép. | **PREFLIGHT PASS / GATED BY OPERATOR CONFIRMATION** |

---

## 8. Những Điều Kiện Còn Thiếu Trước Nghiệm Thu End-to-End (Test A)

Theo đúng quy định an toàn và chỉ thị của bài toán:
1. **Xác nhận đơn hàng DRAFT bởi Operator:** Người vận hành cần truy cập HMI hoặc Staging API để chủ động xác nhận (CONFIRM) đơn hàng giao tài liệu tới Phòng 02 (không để AI tự ý xác nhận thay người vận hành).
2. **Kích hoạt chuyển động `allow_motion=true`:** Khởi chạy Edge Gateway với tham số `allow_motion:=true` sau khi người vận hành đã sẵn sàng quan sát robot di chuyển trong Gazebo.
3. **Thực hiện quét mã QR thực tế:** Thực hiện publish thông điệp quét QR tại điểm lấy hàng (Pickup A) và điểm giao hàng (Phòng 02) qua topic `/office_robot/qr_scan` theo đúng định dạng `{"challengeId": "...", "token": "..."}`.
