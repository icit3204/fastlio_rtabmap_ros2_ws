"""Passive lifecycle/validity observer for future formal episodes."""
import argparse, json, os, time
from pathlib import Path
import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from lifecycle_msgs.msg import TransitionEvent
from lifecycle_msgs.srv import GetState
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool


class Observer:
    def __init__(self, out):
        self.out = Path(out); self.out.mkdir(parents=True, exist_ok=True)
        self.events = self.out / "lifecycle_observer.jsonl"
        self.node = rclpy.create_node("b2u_passive_lifecycle_observer")
        self.client = self.node.create_client(GetState, "/collision_monitor/get_state")
        self.node.create_subscription(TransitionEvent, "/collision_monitor/transition_event", self.transition, 50)
        self.node.create_subscription(DiagnosticArray, "/diagnostics", self.diag, 50)
        self.node.create_subscription(Bool, "/system/collision_monitor_valid", self.bool, 50)
        self.node.create_subscription(LaserScan, "/phase4/synthetic_scan", self.scan, 50)

    def emit(self, event, **fields):
        with self.events.open("a") as f:
            f.write(json.dumps({"monotonic_ns": time.monotonic_ns(), "event": event, **fields}, sort_keys=True) + "\n")
            f.flush(); os.fsync(f.fileno())

    def transition(self, msg):
        self.emit("LIFECYCLE_TRANSITION", start=msg.start_state.label, goal=msg.goal_state.label,
                  start_id=int(msg.start_state.id), goal_id=int(msg.goal_state.id))

    def diag(self, msg):
        for s in msg.status:
            if s.name == "vehicle_cmd_safety/collision_monitor_validity_monitor":
                self.emit("VALIDITY_DIAGNOSTIC", level=int(s.level), message=s.message,
                          values={x.key: x.value for x in s.values})

    def bool(self, msg): self.emit("VALIDITY_BOOL", value=bool(msg.data))
    def scan(self, msg): self.emit("SOURCE_SCAN", count=len(msg.ranges), frame_id=msg.header.frame_id)

    def poll(self):
        state = None; available = self.client.service_is_ready()
        if available:
            fut = self.client.call_async(GetState.Request())
            rclpy.spin_until_future_complete(self.node, fut, timeout_sec=0.15)
            if fut.done() and fut.exception() is None:
                state = {"id": int(fut.result().current_state.id), "label": fut.result().current_state.label}
        self.emit("LIFECYCLE_POLL", service_available=available, state=state)

    def close(self): self.node.destroy_node()


def main():
    ap = argparse.ArgumentParser(); ap.add_argument("--output-dir", required=True); a = ap.parse_args()
    rclpy.init(args=[]); o = Observer(a.output_dir)
    o.emit("OBSERVER_READY", domain=os.environ.get("ROS_DOMAIN_ID"))
    (Path(a.output_dir) / "READY.json").write_text(json.dumps({"event": "OBSERVER_READY", "monotonic_ns": time.monotonic_ns()}) + "\n")
    try:
        while rclpy.ok():
            try:
                rclpy.spin_once(o.node, timeout_sec=0.05)
                o.poll()
            except Exception:
                break
            time.sleep(0.05)
    except KeyboardInterrupt: pass
    finally:
        o.close()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__": main()
