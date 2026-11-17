"""The bridge log decides whether `up xr` really has a headset (lines copied from real runs)."""
from launch_helpers.devices.xr import FAILED, PENDING, READY, judge

SELECTED = "[INFO] [xr_target_node]: 头显选择：PA9410MGL7110258G\n"
CONNECTED = "[INFO] [xr_target_node]: SDK 事件: PC Service 已连接 \n"
FOUND = "[INFO] [xr_target_node]: SDK 事件: 发现头显 PA9410MGL7110258G\n"
DATA = "[INFO] [xr_target_node]: 接收头显追踪数据，设备 ID: PA9410MGL7110258G\n"
SERVICE_DOWN = "[WARN] [xr_target_node]: [XR-002] PC Service 未连接，重新初始化 XRoboToolkit SDK ...\n"


def test_the_first_headset_frame_means_ready_and_names_the_headset():
    verdict = judge(SELECTED + CONNECTED + FOUND + DATA, alive=True)
    assert verdict.state == READY and verdict.reasons == ["PA9410MGL7110258G"]


def test_connected_service_without_headset_waits_and_says_which_headset():
    verdict = judge(SELECTED + CONNECTED, alive=True)
    assert verdict.state == PENDING
    assert "没有收到头显发来的追踪数据" in verdict.reasons[0] and "PA9410MGL7110258G" in verdict.reasons[0]


def test_without_a_selected_headset_the_message_stays_generic():
    verdict = judge("[INFO] [xr_target_node]: 头显选择：等待首台发送追踪数据的头显\n" + CONNECTED, alive=True)
    assert verdict.state == PENDING and "没有收到头显发来的追踪数据" in verdict.reasons[0]


def test_no_service_connection_is_reported_with_the_bridge_errors():
    verdict = judge(SELECTED + SERVICE_DOWN, alive=True)
    assert verdict.state == PENDING
    assert "还没有连上 PC Service" in verdict.reasons[0]
    assert any("[XR-002]" in line for line in verdict.reasons)


def test_an_exited_bridge_is_a_failure_even_if_it_once_had_data():
    assert judge(SELECTED + CONNECTED + DATA, alive=False).state == FAILED
    assert judge("", alive=False).reasons[0] == "XR 桥进程已退出"


def test_nothing_logged_yet_is_pending():
    assert judge("", alive=True).state == PENDING
