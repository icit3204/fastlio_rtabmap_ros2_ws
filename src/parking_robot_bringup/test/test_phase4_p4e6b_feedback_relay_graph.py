"""Installed-Humble isolated action graph qualification; no Nav2 runtime."""
import threading
import time

import pytest

rclpy = pytest.importorskip("rclpy")
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient, ActionServer, CancelResponse
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from parking_robot_bringup.phase4_p4e6b_feedback_relay import (
    CANONICAL_FEEDBACK, SHADOW_FEEDBACK, FeedbackRelay)


def endpoints(node):
    name, namespace=node.get_name(),node.get_namespace()
    return {
        "publishers": node.get_publisher_names_and_types_by_node(name,namespace),
        "subscribers": node.get_subscriber_names_and_types_by_node(name,namespace),
        "services": node.get_service_names_and_types_by_node(name,namespace),
        "clients": node.get_client_names_and_types_by_node(name,namespace),
    }


def test_feedback_only_remap_endpoint_cardinality():
    rclpy.init()
    server_node=Node("p4e6b1a_stub_cardinality")
    client_node=Node("p4e6b1a_mm_cardinality",cli_args=["--ros-args","-r",
        CANONICAL_FEEDBACK+":="+SHADOW_FEEDBACK],use_global_arguments=False)
    async def execute(gh): gh.succeed(); return NavigateToPose.Result()
    server=ActionServer(server_node,NavigateToPose,"/navigate_to_pose",execute_callback=execute)
    client=ActionClient(client_node,NavigateToPose,"/navigate_to_pose")
    relay=FeedbackRelay()
    se,ce,re=endpoints(server_node),endpoints(client_node),endpoints(relay)
    assert SHADOW_FEEDBACK in dict(ce["subscribers"])
    assert CANONICAL_FEEDBACK not in dict(ce["subscribers"])
    assert "/navigate_to_pose/_action/status" in dict(ce["subscribers"])
    canonical=["/navigate_to_pose/_action/send_goal","/navigate_to_pose/_action/get_result","/navigate_to_pose/_action/cancel_goal"]
    assert all(x in dict(ce["clients"]) and x in dict(se["services"]) for x in canonical)
    assert all(x not in dict(re["services"]) and x not in dict(re["clients"]) for x in canonical)
    assert CANONICAL_FEEDBACK in dict(se["publishers"])
    assert SHADOW_FEEDBACK in dict(re["publishers"])
    assert "/navigate_to_pose/_action/status" not in dict(re["publishers"])
    relay.destroy_node(); client.destroy(); server.destroy()
    client_node.destroy_node(); server_node.destroy_node(); rclpy.shutdown()


def test_forward_suppress_uuid_contents_cancel_and_result_direct():
    rclpy.init()
    server_node=Node("p4e6b1a_stub_flow")
    client_node=Node("p4e6b1a_mm_flow",cli_args=["--ros-args","-r",
        CANONICAL_FEEDBACK+":="+SHADOW_FEEDBACK],use_global_arguments=False)
    relay=FeedbackRelay(); received=[]; canonical=[]; canceled=[]
    observer=server_node.create_subscription(NavigateToPose.Impl.FeedbackMessage,
        CANONICAL_FEEDBACK,lambda m: canonical.append(m),10)
    def execute(gh):
        for value in (3.0,2.0,1.0):
            msg=NavigateToPose.Feedback(); msg.distance_remaining=value
            msg.number_of_recoveries=int(value); gh.publish_feedback(msg)
            time.sleep(.03)
        while not gh.is_cancel_requested: time.sleep(.01)
        canceled.append(bytes(gh.goal_id.uuid)); gh.canceled(); return NavigateToPose.Result()
    server=ActionServer(server_node,NavigateToPose,"/navigate_to_pose",
        execute_callback=execute,cancel_callback=lambda _:CancelResponse.ACCEPT)
    client=ActionClient(client_node,NavigateToPose,"/navigate_to_pose")
    executor=MultiThreadedExecutor(num_threads=4)
    for n in (server_node,client_node,relay): executor.add_node(n)
    thread=threading.Thread(target=executor.spin,daemon=True); thread.start()
    assert client.wait_for_server(2.0)
    goal_future=client.send_goal_async(NavigateToPose.Goal(),feedback_callback=lambda m:received.append(m))
    deadline=time.time()+3
    while not goal_future.done() and time.time()<deadline: time.sleep(.01)
    goal=goal_future.result(); assert goal.accepted
    deadline=time.time()+3
    while len(received)<3 and time.time()<deadline: time.sleep(.01)
    assert [m.feedback.distance_remaining for m in received]==[3.0,2.0,1.0]
    assert [m.feedback.number_of_recoveries for m in received]==[3,2,1]
    assert all(bytes(m.goal_id.uuid)==bytes(goal.goal_id.uuid) for m in received)
    relay.suppress=True; relay.injection_count=1; relay.injection_monotonic_ns=time.monotonic_ns()
    extra=NavigateToPose.Impl.FeedbackMessage(); extra.goal_id=goal.goal_id; extra.feedback.distance_remaining=.5
    relay._feedback(extra); assert len(received)==3
    cancel_future=goal.cancel_goal_async()
    deadline=time.time()+3
    while not cancel_future.done() and time.time()<deadline: time.sleep(.01)
    assert len(cancel_future.result().goals_canceling)==1
    result_future=goal.get_result_async(); deadline=time.time()+3
    while not result_future.done() and time.time()<deadline: time.sleep(.01)
    assert result_future.result().status==GoalStatus.STATUS_CANCELED
    assert canceled==[bytes(goal.goal_id.uuid)] and len(canonical)>=3
    executor.shutdown(); thread.join(2); client.destroy(); server.destroy()
    relay.destroy_node(); client_node.destroy_node(); server_node.destroy_node(); rclpy.shutdown()


@pytest.mark.parametrize("cancel_response",[CancelResponse.ACCEPT,CancelResponse.REJECT])
def test_cancel_response_is_direct_from_canonical_server(cancel_response):
    suffix=cancel_response.name.lower()
    action_name="/p4e6b1a_cancel_"+suffix
    rclpy.init(); server_node=Node("p4e6b1a_cancel_server_"+suffix)
    client_node=Node("p4e6b1a_cancel_client_"+suffix,cli_args=["--ros-args","-r",
        action_name+"/_action/feedback:="+SHADOW_FEEDBACK],use_global_arguments=False)
    def execute(gh):
        deadline=time.time()+2
        while time.time()<deadline and not gh.is_cancel_requested: time.sleep(.01)
        if gh.is_cancel_requested: gh.canceled()
        else: gh.abort()
        return NavigateToPose.Result()
    server=ActionServer(server_node,NavigateToPose,action_name,
        execute_callback=execute,cancel_callback=lambda _:cancel_response)
    client=ActionClient(client_node,NavigateToPose,action_name)
    shadow_publisher=server_node.create_publisher(
        NavigateToPose.Impl.FeedbackMessage,SHADOW_FEEDBACK,10)
    executor=MultiThreadedExecutor(num_threads=3)
    for n in (server_node,client_node): executor.add_node(n)
    thread=threading.Thread(target=executor.spin,daemon=True); thread.start()
    assert client.wait_for_server(5); future=client.send_goal_async(NavigateToPose.Goal())
    deadline=time.time()+2
    while not future.done() and time.time()<deadline: time.sleep(.01)
    goal=future.result(); cancel=goal.cancel_goal_async(); deadline=time.time()+2
    while not cancel.done() and time.time()<deadline: time.sleep(.01)
    assert len(cancel.result().goals_canceling)==(1 if cancel_response==CancelResponse.ACCEPT else 0)
    executor.shutdown(); thread.join(2); client.destroy(); server.destroy()
    client_node.destroy_node(); server_node.destroy_node(); rclpy.shutdown()
