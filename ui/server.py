#!/usr/bin/env python3
import argparse
import json
import os
import shlex
import shutil
import signal
import subprocess
import threading
import time
from collections import OrderedDict
from functools import partial
from http import HTTPStatus
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Dict, Optional

UI_ROOT = Path(__file__).resolve().parent
PROJECT_ROOT = UI_ROOT.parent
RUNTIME_DIR = UI_ROOT / "runtime"


def build_shell_prefix() -> str:
    parts = ["source /opt/ros/noetic/setup.bash"]
    catkin_setup = Path.home() / "catkin_ws" / "devel" / "setup.bash"
    if catkin_setup.exists():
        parts.append(f"source {shlex.quote(str(catkin_setup))}")
    parts.append(f"cd {shlex.quote(str(PROJECT_ROOT))}")
    return "; ".join(parts)


START_COMMANDS = OrderedDict({
    "camera": [
        "bash",
        "scripts/start_camera.sh",
    ],
    "camera_tf": [
        "rosrun",
        "tf",
        "static_transform_publisher",
        "0",
        "0",
        "0",
        "0",
        "0",
        "0",
        "realsense_link",
        "camera_link",
        "100",
    ],
    "vision": [
        "bash",
        "scripts/run_vision.sh",
    ],
    "player_selection": [
        "bash",
        "-lc",
        "rosparam load config/game_params.yaml && rosrun memory_game player_selection.py __name:=player_selection",
    ],
    "game": [
        "bash",
        "-lc",
        "rosparam load config/game_params.yaml && rosrun memory_game game_node __name:=game_node",
    ],
    "motion": [
        "bash",
        "-lc",
        "rosparam load config/game_params.yaml && rosrun memory_game motion_moveit_node __name:=motion_moveit_node _planning_group:=arm",
    ],
})

PROCESS_LABELS = {
    "camera": "RealSense Camera",
    "camera_tf": "Camera TF Bridge",
    "vision": "Vision Node",
    "player_selection": "Player Selection",
    "game": "Game Node",
    "motion": "MoveIt Motion",
}

# Before launching a process, wait until this ROS node is visible in rosnode list.
# This prevents motion from starting before game_node is fully registered.
LAUNCH_DEPENDENCIES: Dict[str, str] = {
    "motion": "/game_node",
}
DEPENDENCY_TIMEOUT_SEC = 40.0


PLAYER_MESSAGE_MAP = {
    "UNKNOWN": {
        "phase": "Connecting",
        "headline": "Connecting to the game",
        "detail": "Waiting for live ROS data from the robot system.",
        "tone": "neutral",
    },
    "IDLE": {
        "phase": "Ready",
        "headline": "Get ready",
        "detail": "The next round will begin shortly.",
        "tone": "neutral",
    },
    "WAITING_FOR_BLOCKS": {
        "phase": "Setup",
        "headline": "Preparing the table",
        "detail": "Waiting until the colored blocks are visible and stable.",
        "tone": "neutral",
    },
    "WAITING_FOR_MOTION": {
        "phase": "Setup",
        "headline": "Preparing the robot",
        "detail": "Waiting for the motion system before the round starts.",
        "tone": "neutral",
    },
    "SHOWING_SEQUENCE": {
        "phase": "Watch",
        "headline": "Watch the sequence",
        "detail": "Memorize the order the robot is showing.",
        "tone": "active",
    },
    "WAITING_PLAYER": {
        "phase": "Your Turn",
        "headline": "Your turn",
        "detail": "Repeat the sequence by moving the matching blocks.",
        "tone": "player",
    },
    "CHECKING_INPUT": {
        "phase": "Checking",
        "headline": "Checking your answer",
        "detail": "Hold on while the system compares your choices.",
        "tone": "active",
    },
    "ROUND_PAUSE": {
        "phase": "Correct",
        "headline": "Nice work",
        "detail": "That round is complete. The next sequence is coming.",
        "tone": "success",
    },
    "MOTION_FAILED": {
        "phase": "Paused",
        "headline": "Robot needs attention",
        "detail": "The team needs to reset the motion side before continuing.",
        "tone": "danger",
    },
    "GAME_OVER": {
        "phase": "Round Over",
        "headline": "Round complete",
        "detail": "The sequence ended. The team can start a new run when ready.",
        "tone": "danger",
    },
}


def iso_now() -> str:
    return time.strftime("%Y-%m-%d %H:%M:%S")


def build_player_message(game_state: str, motion_status: str, score: Optional[int]) -> dict:
    base = PLAYER_MESSAGE_MAP.get(game_state, PLAYER_MESSAGE_MAP["UNKNOWN"]).copy()
    if game_state == "SHOWING_SEQUENCE" and motion_status and motion_status != "UNKNOWN":
        base["detail"] = f"Memorize the order. Robot status: {motion_status}."
    if game_state == "WAITING_PLAYER" and score is not None:
        base["detail"] = f"Repeat the sequence by moving the matching blocks. Current score: {score}."
    return base


class BridgeState:
    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._state = {
            "bridge_status": "starting",
            "bridge_message": "Preparing ROS bridge.",
            "score": None,
            "game_state": "UNKNOWN",
            "motion_status": "UNKNOWN",
            "detected_block_count": 0,
            "detected_blocks": [],
            "player_selection": None,
            "player_progress": "",
            "player_message": build_player_message("UNKNOWN", "UNKNOWN", None),
            "process_status": {},
            "last_update": None,
            "log": [
                {"time": iso_now(), "message": "UI bridge process started."},
            ],
        }

    def snapshot(self):
        with self._lock:
            return json.loads(json.dumps(self._state))

    def update(self, **kwargs):
        with self._lock:
            self._state.update(kwargs)
            self._state["player_message"] = build_player_message(
                self._state.get("game_state", "UNKNOWN"),
                self._state.get("motion_status", "UNKNOWN"),
                self._state.get("score"),
            )
            self._state["last_update"] = iso_now()

    def append_log(self, message: str):
        with self._lock:
            self._state["log"].insert(0, {"time": iso_now(), "message": message})
            self._state["log"] = self._state["log"][:16]
            self._state["last_update"] = iso_now()


class ProcessManager:
    def __init__(self, state: BridgeState) -> None:
        self.state = state
        self._lock = threading.Lock()
        self._procs: Dict[str, subprocess.Popen] = {}
        self._starting = False
        self._terminal_cmd = self._detect_terminal()
        RUNTIME_DIR.mkdir(exist_ok=True)
        self.refresh_state()

    def _detect_terminal(self):
        for candidate in ("gnome-terminal", "x-terminal-emulator"):
            path = shutil.which(candidate)
            if path:
                return path
        return None

    def _launch_in_terminal(self, name: str, cmd):
        if not self._terminal_cmd:
            raise RuntimeError("No supported terminal emulator found (expected gnome-terminal or x-terminal-emulator)")

        log_path = RUNTIME_DIR / f"{name}.log"
        shell_prefix = build_shell_prefix()
        command_text = shlex.join(cmd)
        log_text = shlex.quote(str(log_path))
        shell_script = (
            f"{shell_prefix}; "
            f"echo '[{name}] {command_text}'; "
            f"set -o pipefail; {command_text} 2>&1 | tee -a {log_text}; "
            "status=${PIPESTATUS[0]}; "
            'echo; echo "Exit code: $status"; '
            "exec bash"
        )

        if os.path.basename(self._terminal_cmd) == "gnome-terminal":
            terminal_cmd = [
                self._terminal_cmd,
                "--title",
                f"memory_game_{name}",
                "--",
                "bash",
                "-lc",
                shell_script,
            ]
        else:
            terminal_cmd = [
                self._terminal_cmd,
                "-T",
                f"memory_game_{name}",
                "-e",
                "bash",
                "-lc",
                shell_script,
            ]

        proc = subprocess.Popen(terminal_cmd, cwd=str(PROJECT_ROOT), start_new_session=True)
        self.state.append_log(f"Opened terminal for {name} (pid={proc.pid}).")
        return proc

    def _status_for(self, name: str) -> dict:
        proc = self._procs.get(name)
        log_path = str(RUNTIME_DIR / f"{name}.log")
        label = PROCESS_LABELS.get(name, name)
        if proc is None:
            return {"label": label, "running": False, "pid": None, "log_path": log_path}
        running = proc.poll() is None
        return {"label": label, "running": running, "pid": proc.pid, "log_path": log_path}

    def refresh_state(self) -> None:
        self.state.update(process_status={name: self._status_for(name) for name in START_COMMANDS})

    def _wait_for_ros_node(self, node_name: str, timeout: float) -> bool:
        """Poll rosnode list until node_name appears or timeout expires."""
        shell_prefix = build_shell_prefix()
        deadline = time.time() + timeout
        while time.time() < deadline:
            try:
                result = subprocess.run(
                    ["bash", "-lc", f"{shell_prefix}; rosnode list"],
                    capture_output=True, text=True, timeout=5,
                )
                if node_name in result.stdout.split():
                    return True
            except Exception:
                pass
            time.sleep(1.5)
        return False

    def _sequenced_start(self) -> None:
        """Run in a background thread — start processes in dependency order."""
        try:
            for name, cmd in START_COMMANDS.items():
                dep_node = LAUNCH_DEPENDENCIES.get(name)
                if dep_node:
                    label = PROCESS_LABELS.get(name, name)
                    self.state.append_log(f"Waiting for {dep_node} before starting {label}...")
                    self.state.update(bridge_message=f"Waiting for {dep_node} to be ready...")
                    if not self._wait_for_ros_node(dep_node, DEPENDENCY_TIMEOUT_SEC):
                        msg = f"Startup aborted: {dep_node} did not appear within {DEPENDENCY_TIMEOUT_SEC:.0f}s. Check the game node terminal."
                        self.state.append_log(msg)
                        self.state.update(bridge_message=msg)
                        with self._lock:
                            self._stop_locked()
                        self.refresh_state()
                        return
                    self.state.append_log(f"{dep_node} is up — launching {label}.")

                log_path = RUNTIME_DIR / f"{name}.log"
                with open(log_path, "a", encoding="utf-8") as log_file:
                    log_file.write(f"\n[{iso_now()}] Starting in terminal: {' '.join(cmd)}\n")
                try:
                    proc = self._launch_in_terminal(name, cmd)
                except Exception as exc:
                    msg = f"Failed to launch {name}: {exc}"
                    self.state.append_log(msg)
                    self.state.update(bridge_message=msg)
                    with self._lock:
                        self._stop_locked()
                    self.refresh_state()
                    return

                with self._lock:
                    self._procs[name] = proc
                self.state.append_log(f"Started {PROCESS_LABELS.get(name, name)}.")
                self.refresh_state()
                time.sleep(0.25)

            self.state.update(bridge_message="All processes started.")
            self.state.append_log("Session fully started.")
        finally:
            with self._lock:
                self._starting = False
            self.refresh_state()

    def start_all(self) -> dict:
        with self._lock:
            already_running = [name for name, proc in self._procs.items() if proc.poll() is None]
            if already_running or self._starting:
                self.refresh_state()
                if self._starting:
                    return {"ok": False, "message": "Startup already in progress."}
                labels = [PROCESS_LABELS.get(name, name) for name in sorted(already_running)]
                return {
                    "ok": False,
                    "message": f"Processes already running: {', '.join(labels)}",
                }
            self._starting = True

        threading.Thread(target=self._sequenced_start, daemon=True).start()
        return {"ok": True, "message": "Session startup initiated — processes will start in sequence."}

    def _stop_locked(self) -> None:
        for proc in list(self._procs.values()):
            if proc.poll() is None:
                try:
                    os.killpg(proc.pid, signal.SIGTERM)
                except ProcessLookupError:
                    pass
        deadline = time.time() + 3.0
        while time.time() < deadline:
            if all(proc.poll() is not None for proc in self._procs.values()):
                break
            time.sleep(0.1)
        for proc in list(self._procs.values()):
            if proc.poll() is None:
                try:
                    os.killpg(proc.pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
        self._procs.clear()

    def stop_all(self) -> dict:
        with self._lock:
            if not self._procs:
                self.refresh_state()
                return {"ok": True, "message": "No managed processes were running."}
            self._stop_locked()
            self.state.append_log("Stopped managed session processes.")
            self.refresh_state()
            return {"ok": True, "message": "Stopped camera, TF bridge, vision, selection, game, and motion processes."}

    def _kill_one(self, name: str) -> None:
        proc = self._procs.get(name)
        if proc is None or proc.poll() is not None:
            return
        try:
            os.killpg(proc.pid, signal.SIGTERM)
        except ProcessLookupError:
            pass
        deadline = time.time() + 3.0
        while time.time() < deadline:
            if proc.poll() is not None:
                break
            time.sleep(0.1)
        if proc.poll() is None:
            try:
                os.killpg(proc.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass

    def restart_game(self) -> dict:
        with self._lock:
            self._kill_one("game")
            name = "game"
            cmd = START_COMMANDS[name]
            log_path = RUNTIME_DIR / f"{name}.log"
            with open(log_path, "a", encoding="utf-8") as f:
                f.write(f"\n[{iso_now()}] Restarting game node\n")
            try:
                proc = self._launch_in_terminal(name, cmd)
                self._procs[name] = proc
            except Exception as exc:
                self.state.append_log(f"Game restart failed: {exc}")
                self.refresh_state()
                return {"ok": False, "message": f"Failed to restart game: {exc}"}
        self.state.append_log("Game node restarted.")
        self.refresh_state()
        return {"ok": True, "message": "Game restarted."}


class RosBridge:
    def __init__(self, state: BridgeState) -> None:
        self.state = state
        self._sequence_length = 0
        self._selection_count = 0
        self.thread = threading.Thread(target=self._run, daemon=True)

    def start(self) -> None:
        self.thread.start()

    def _run(self) -> None:
        try:
            import rospy
            from std_msgs.msg import Int32, String
            from memory_game.msg import BlockArray, BlockSequence, PlayerSelection
        except Exception as exc:  # pragma: no cover - runtime environment dependent
            self.state.update(
                bridge_status="error",
                bridge_message=(
                    "ROS Python packages are not available. Source the catkin workspace "
                    "before running ui/server.py."
                ),
            )
            self.state.append_log(f"Bridge import failed: {exc}")
            return

        self.state.update(
            bridge_status="connecting",
            bridge_message="Connecting to ROS master and subscribing to topics.",
        )

        try:
            rospy.init_node("memory_game_ui_bridge", anonymous=True, disable_signals=True)
        except Exception as exc:  # pragma: no cover - runtime environment dependent
            self.state.update(
                bridge_status="error",
                bridge_message="Could not initialize rospy. Check ROS master and environment.",
            )
            self.state.append_log(f"rospy.init_node failed: {exc}")
            return

        self.state.update(
            bridge_status="connected",
            bridge_message="Live ROS topic bridge is active.",
        )
        self.state.append_log("Subscribed to /score, /game_state, /motion_status, /detected_blocks, /player_selection, /target_sequence.")

        rospy.Subscriber("/score", Int32, self._score_cb, queue_size=5)
        rospy.Subscriber("/game_state", String, self._game_state_cb, queue_size=5)
        rospy.Subscriber("/motion_status", String, self._motion_cb, queue_size=5)
        rospy.Subscriber("/detected_blocks", BlockArray, self._blocks_cb, queue_size=5)
        rospy.Subscriber("/player_selection", PlayerSelection, self._selection_cb, queue_size=5)
        rospy.Subscriber("/target_sequence", BlockSequence, self._target_sequence_cb, queue_size=5)

        rospy.spin()

    def _score_cb(self, msg):
        self.state.update(score=msg.data)

    def _game_state_cb(self, msg):
        self.state.update(game_state=msg.data)
        if msg.data != "WAITING_PLAYER":
            self._selection_count = 0
            self._update_progress()

    def _motion_cb(self, msg):
        self.state.update(motion_status=msg.data)

    def _blocks_cb(self, msg):
        blocks = []
        for block in msg.blocks:
            blocks.append({
                "id": int(block.id),
                "color": block.color,
                "position": {
                    "x": round(float(block.position.x), 3),
                    "y": round(float(block.position.y), 3),
                    "z": round(float(block.position.z), 3),
                },
                "confidence": round(float(block.confidence), 3),
            })
        self.state.update(detected_blocks=blocks, detected_block_count=len(blocks))

    def _target_sequence_cb(self, msg):
        self._sequence_length = len(msg.blocks)
        self._selection_count = 0
        self._update_progress()

    def _update_progress(self):
        if self._sequence_length > 0:
            self.state.update(player_progress=f"{self._selection_count}/{self._sequence_length}")
        else:
            self.state.update(player_progress="")

    def _selection_cb(self, msg):
        self._selection_count += 1
        self._update_progress()
        self.state.update(player_selection={
            "block_id": int(msg.block_id),
            "color": msg.color,
            "selection_type": msg.selection_type,
            "confidence": round(float(msg.confidence), 3),
        })


class UiHandler(SimpleHTTPRequestHandler):
    def __init__(self, *args, directory=None, bridge_state=None, process_manager=None, **kwargs):
        self.bridge_state = bridge_state
        self.process_manager = process_manager
        super().__init__(*args, directory=directory, **kwargs)

    def _write_json(self, payload: dict, status: int = HTTPStatus.OK) -> None:
        body = json.dumps(payload).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        if self.path == "/api/status":
            self.process_manager.refresh_state()
            self._write_json(self.bridge_state.snapshot())
            return
        return super().do_GET()

    def do_POST(self):
        if self.path == "/api/start":
            result = self.process_manager.start_all()
            self._write_json(result, HTTPStatus.OK if result.get("ok") else HTTPStatus.BAD_REQUEST)
            return
        if self.path == "/api/stop":
            result = self.process_manager.stop_all()
            self._write_json(result)
            return
        if self.path == "/api/restart_game":
            result = self.process_manager.restart_game()
            self._write_json(result, HTTPStatus.OK if result.get("ok") else HTTPStatus.BAD_REQUEST)
            return
        self._write_json({"ok": False, "message": "Unknown endpoint"}, HTTPStatus.NOT_FOUND)

    def log_message(self, format, *args):
        return


def main() -> None:
    parser = argparse.ArgumentParser(description="Serve the memory-game UI and bridge simple ROS topics.")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8000)
    args = parser.parse_args()

    state = BridgeState()
    process_manager = ProcessManager(state)
    bridge = RosBridge(state)
    bridge.start()

    handler = partial(
        UiHandler,
        directory=str(UI_ROOT),
        bridge_state=state,
        process_manager=process_manager,
    )
    httpd = ThreadingHTTPServer((args.host, args.port), handler)
    print(f"Serving UI from {UI_ROOT} on http://{args.host}:{args.port}")
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        httpd.server_close()


if __name__ == "__main__":
    main()
