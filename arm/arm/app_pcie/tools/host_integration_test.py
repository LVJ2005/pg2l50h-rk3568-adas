#!/usr/bin/env python3
"""Host-only integration checks, using app_host_test and the RKNN test double."""
from __future__ import annotations

import importlib.util
import os
from pathlib import Path
import socket
import subprocess
import tempfile
import threading
import time

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]


def command(port: int, text: str) -> str:
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        sock.settimeout(.4)
        sock.connect(("127.0.0.1", port))
        sock.send(text.encode("ascii"))
        return sock.recv(8192).decode("ascii")


def values(port: int) -> dict[str, str]:
    return dict(item.split("=", 1) for item in command(port, "STATUS").split() if "=" in item)


def wait_for(port: int, predicate, timeout: float = 6) -> dict[str, str]:
    end = time.monotonic() + timeout
    last = {}
    last_error = None
    while time.monotonic() < end:
        try:
            last = values(port)
        except OSError as error:
            last_error=error
            time.sleep(.05)
            continue
        if predicate(last):
            return last
        time.sleep(.05)
    raise AssertionError((last,str(last_error) if last_error else None))


def create_video(path: Path) -> None:
    video = cv2.VideoWriter(str(path), cv2.VideoWriter_fourcc(*"MJPG"), 25, (1280, 720))
    assert video.isOpened()
    frame = np.full((720, 1280, 3), 45, dtype=np.uint8)
    for quadrant in range(4):
        ox, oy = (quadrant % 2) * 640, (quadrant // 2) * 360
        if quadrant:
            frame[oy:oy+360,ox:ox+640]=(60+quadrant*20,30+quadrant*15,100+quadrant*20)
            continue
        for y in range(170, 353):
            t = (y - 162) / 190
            left = int(640 * (.38 - .22 * t))
            right = int(640 * (.62 + .22 * t))
            frame[oy+y, ox+left-5:ox+left+6] = 240
            if (y // 18) % 2 == 0:
                frame[oy+y, ox+right-5:ox+right+6] = 240
    for _ in range(100):
        video.write(frame)
    video.release()


def gui_worker_test() -> None:
    spec = importlib.util.spec_from_file_location("adas_control", ROOT / "tools/recognition_control.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    shown = module.describe_status({
        "action": "straight", "lane_action": "lane_left", "signal_state": "allow",
        "lane_state": "allow", "pedestrian_state": "stop", "person_seen": "1",
        "crosswalk_seen": "1", "person_on_crosswalk": "1",
    })
    assert len(shown) == 7 and "紧急制动" in shown[0] and "绿灯" in shown[4]
    right = module.describe_status({"action": "right", "lane_action": "stop",
                                    "signal_state": "allow", "signal_reason": "right_turn_yield"})
    assert "须确认让行" in right[4] and "须确认让行" in right[0]
    assert "12.5 ms" in module.describe_latency({
        "pre": "1.2", "det_npu": "8.5", "cls_npu": "2.8", "lane_ms": "4.1"})
    server = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    server.bind(("127.0.0.1", 0))
    server.settimeout(.1)
    stopped = threading.Event()

    def slow_peer():
        while not stopped.is_set():
            try:
                _, peer = server.recvfrom(1024)
            except (socket.timeout, ConnectionResetError):
                continue
            time.sleep(.15)
            server.sendto(b"OK", peer)

    thread = threading.Thread(target=slow_peer)
    thread.start()
    worker = module.NetworkWorker("127.0.0.1", server.getsockname()[1], .3)
    try:
        start = time.monotonic()
        worker.drive("left")
        worker.submit("STATUS")
        assert time.monotonic() - start < .05
        time.sleep(.6)
        assert not worker.events.empty()
    finally:
        worker.close()
        worker.thread.join(1)
        stopped.set()
        thread.join(1)
        server.close()


def main() -> None:
    gui_worker_test()
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as probe:
        probe.bind(("127.0.0.1", 0))
        port = probe.getsockname()[1]
    with tempfile.TemporaryDirectory(prefix="adas-test-") as folder:
        folder = Path(folder)
        video = folder / "quad.avi"
        create_video(video)
        environment = dict(os.environ, RECOGNITION_CONTROL_BIND="127.0.0.1",
                           RECOGNITION_CONTROL_PORT=str(port), ADAS_TEST_NPU_MS="400")
        environment.pop("ADAS_SIGNAL_CONFIRMED",None)
        environment.pop("ADAS_LANE_SCAN_ROWS",None)
        display_test = os.environ.get("ADAS_TEST_DISPLAY") == "1"
        environment["ADAS_DISPLAY"] = "1" if display_test else "0"
        environment["ADAS_LANE_DEBUG"] = "1" if display_test else "0"
        environment["ADAS_COLOR_COMPARE"] = "1" if display_test else "0"
        environment["ADAS_DEBUG_DIR"] = str(folder / "snapshots")
        with (folder / "app.log").open("w") as log:
            app = subprocess.Popen([str(ROOT / "build/app_host_test"), "--models", str(ROOT / "models"),
                                    "--video", str(video)], stdout=log, stderr=log, env=environment)
            try:
                time.sleep(.4)
                wait_for(port, lambda s: s.get("loaded_det") == "fp" and s.get("detections") == "1")
                wait_for(port, lambda s: s.get("lane_left") == "solid" and s.get("lane_right") == "dashed")
                assert values(port)["display_order"] == "rgb" and values(port)["input_order"] == "bgr"
                first_status=values(port)
                assert first_status["version"]=="ADAS-quad-V2.0.10"
                assert first_status["pedestrian_enabled"]=="1" and first_status["brake_request"]=="0"
                assert len(command(port,"STATUS").encode("ascii"))<1400
                assert len(command(port,"STATUS CONFIG").encode("ascii"))<1400
                assert len(command(port,"STATUS LANE").encode("ascii"))<1400
                lane_timing = dict(item.split("=", 1) for item in
                                   command(port,"STATUS LANE").split() if "=" in item)
                stages = [float(lane_timing[key]) for key in
                          ("lane_pre_ms", "lane_mask_ms", "lane_seed_ms", "lane_fit_ms")]
                assert min(stages) >= 0 and sum(stages) <= float(lane_timing["lane_ms"]) + .01
                assert float(lane_timing["lane_cpu_ms"]) >= 0
                assert first_status["signal_confirmed"]=="1" and first_status["lane_scan_rows"]=="100"
                assert float(first_status["lane_occlusion"])==0
                assert first_status["lane_left_fitted"]==first_status["lane_right_fitted"]=="1"
                assert first_status["view_order"]==first_status["infer_order"]=="rbg"
                assert first_status["detection_view"]==first_status["front_view"]=="0"
                assert first_status["detection_scale"]=="1"
                assert command(port,"SET front_view 2").startswith("ERR")
                initial_generation=values(port)["generation"]
                assert command(port,"SET view_order rbg").startswith("OK")
                assert command(port,"SET color_compare 1").startswith("OK")
                assert values(port)["generation"]==initial_generation
                assert command(port,"SNAPSHOT").startswith("OK")
                wait_for(port,lambda s:s.get("snapshot_completed")=="1")
                snapshot=next((folder / "snapshots").iterdir())
                assert (snapshot / "dma_snapshot.bin").stat().st_size==1280*720*3
                raw=cv2.imread(str(snapshot / "raw_bgr.png"))
                raw_bytes=np.fromfile(snapshot / "dma_snapshot.bin",np.uint8).reshape(720,1280,3)
                assert np.array_equal(raw,raw_bytes)
                corrected=cv2.imread(str(snapshot / "display_bgr.png"))
                assert np.array_equal(corrected,raw[:,:,[1,2,0]])
                q1=cv2.imread(str(snapshot / "inference_q1_bgr.png"))
                assert q1.shape==(360,640,3) and np.array_equal(q1,raw[:360,:640,[1,2,0]])
                lane_roi=cv2.imread(str(snapshot / "lane_roi_bgr.png"))
                lane_top=round(float(first_status["lane_top"])*360)
                lane_bottom=round(float(first_status["lane_bottom"])*360)
                assert np.array_equal(lane_roi,q1[lane_top:lane_bottom])
                assert cv2.imread(str(snapshot / "lane_mask.png")).shape[:2]==(100,320)
                assert cv2.imread(str(snapshot / "lane_candidates.png")).shape==(100,320,3)
                assert "lane_trapezoid_roi_relative=" in (snapshot / "metadata.txt").read_text()
                tensor_preview=cv2.imread(str(snapshot / "detector_input.png"))
                assert np.array_equal(tensor_preview[12:372],q1)
                assert np.all(tensor_preview[:12]==114) and np.all(tensor_preview[372:]==114)
                assert command(port,"SET camera_order rbg").startswith("OK")
                config = dict(item.split("=",1) for item in command(port,"STATUS CONFIG").split() if "=" in item)
                assert config["lane_trap_tl"]=="0.280" and config["lane_trap_br"]=="0.980"
                generation=values(port)["generation"]
                assert command(port,"LANE_TRAP .9 .2 0 1").startswith("ERR")
                assert values(port)["generation"]==generation
                assert command(port,"LANE_TRAP .28 .72 .02 .98").startswith("OK")
                assert int(values(port)["generation"])==int(generation)+1
                wait_for(port,lambda s:s.get("lane_left")=="solid" and s.get("lane_right")=="dashed")
                assert command(port, "DRIVE straight 1 hosttest").startswith("OK")
                assert command(port, "DRIVE right 1 hosttest").startswith("ERR")
                before = values(port)
                latencies = []
                for _ in range(15):
                    start = time.monotonic()
                    command(port, "PING")
                    latencies.append(time.monotonic() - start)
                    time.sleep(.02)
                after = values(port)
                assert int(after["frames"]) > int(before["frames"]) + 4
                assert max(latencies) < .25, latencies
                assert float(after["lane_ms"]) >= 0 and int(after["skipped"]) > 0
                wait_for(port, lambda s: s.get("state") == "allow", 1.2)
                assert command(port, "MODEL det int8").startswith("OK")
                assert command(port, "MODEL cls int8").startswith("OK")
                wait_for(port, lambda s: s.get("loaded_det") == "int8" and s.get("loaded_cls") == "int8")
                wait_for(port, lambda s: s.get("reason") == "command_expired", 3)
                assert command(port, "MODE off").startswith("OK")
                assert values(port)["state"] == "idle"
                # Perception failures must not disable the lane/control/capture threads.
                models = ROOT / "models"
                original = models / "det-v6-640x384-fp.rknn"
                hidden = original.with_suffix(".rknn.test_hidden")
                assert not hidden.exists()
                original.rename(hidden)
                try:
                    assert command(port, "MODEL det fp").startswith("OK")
                    assert command(port, "MODE assist").startswith("OK")
                    # A successful recovery clears model_error; verify the persistent rollback state.
                    failed = wait_for(port, lambda s: s.get("det_model")=="int8" and s.get("loaded_det")=="int8")
                    assert failed["det_model"] == "int8" and failed["loaded_det"] == "int8"
                    assert command(port, "PING") == "OK PONG"
                finally:
                    hidden.rename(original)
                print("PASS UDP responsive during 400ms inference, continuing capture/lane, model switching, expiry, rollback")
                print(f"Host-only UDP max={max(latencies)*1000:.2f}ms; these are not RK3568 performance results")
            except Exception:
                print((folder / "app.log").read_text())
                raise
            finally:
                app.terminate()
                try:
                    app.wait(timeout=4)
                except subprocess.TimeoutExpired:
                    app.kill()
                    app.wait()
                if app.returncode not in (0, -15):
                    print((folder / "app.log").read_text())
                    raise AssertionError(app.returncode)


if __name__ == "__main__":
    main()
