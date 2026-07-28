# main.py - MeshScan Unit V entry point.
#
# Pipeline per loop iteration:
#   1. Capture a QVGA frame from the OV2640.
#   2. Run YOLOv2 on the KPU -> list of detections (empty if no model loaded).
#   3. JPEG-compress the frame.
#   4. Send a FRAME packet, then the paired DETECT packet, over UART.
#
# The M5Tab drives the experience (start/stop scan, orbit, save). The Unit V is
# a dumb sensor: it streams frame+detections continuously whenever powered.
# Launched by /flash/boot.py (the MeshScan launcher that replaces the StickV
# factory demo). Streams a live feed even with no detection model present.

import sensor
import image
import time
from machine import UART
from fpioa_manager import fm

import config
from detector import Detector
import uart_protocol as proto


def init_camera():
    # Sensor reset can fail the first few attempts on the Unit V — retry.
    for _ in range(20):
        try:
            sensor.reset()
            break
        except Exception:
            time.sleep_ms(100)
    sensor.set_pixformat(sensor.RGB565)      # KPU + JPEG both work from RGB565
    sensor.set_framesize(sensor.QVGA)        # 320x240
    sensor.set_hmirror(False)
    sensor.set_vflip(False)
    sensor.skip_frames(time=300)             # let AGC/AWB settle


def init_uart():
    # Map the grove pads to the UART2 peripheral.
    fm.register(config.UART_TX_PIN, fm.fpioa.UART2_TX, force=True)
    fm.register(config.UART_RX_PIN, fm.fpioa.UART2_RX, force=True)
    return UART(UART.UART2, config.UART_BAUD, 8, 0, 0, timeout=1000, read_buf_len=256)


def main():
    init_camera()
    uart = init_uart()
    detector = Detector()            # camera-only if no kmodel is present
    print("[main] streaming %dx%d @ %d baud, detect=%s" % (
        config.FRAME_WIDTH, config.FRAME_HEIGHT, config.UART_BAUD,
        "on" if detector.available else "off"))

    frame_id = 0
    clock = time.clock()
    detect_n = getattr(config, "DETECT_EVERY_N", 1)
    detections = []          # last inference result, re-sent between runs

    while True:
        clock.tick()
        img = sensor.snapshot()

        # 1) run inference every Nth frame only (it dominates loop time);
        #    intermediate frames carry the previous detections.
        if frame_id % detect_n == 0:
            detections = detector.detect(img)
            if detections:
                # loud detection telemetry: name what the model sees
                print("[det] fid=%d:" % frame_id, " ".join(
                    "%s=%d%%" % (config.CLASS_NAMES[d.class_id]
                                 if d.class_id < len(config.CLASS_NAMES)
                                 else str(d.class_id), d.confidence)
                    for d in detections[:5]))

        # 2) compress. img.compress() mutates into JPEG in place, so do it last.
        jpeg = img.compress(quality=config.JPEG_QUALITY)
        jpeg_bytes = jpeg.to_bytes()

        # 3) FRAME then paired DETECT, back to back
        uart.write(proto.pack_frame(jpeg_bytes))
        uart.write(proto.pack_detect(frame_id, (d.as_tuple() for d in detections)))

        frame_id = (frame_id + 1) & 0xFFFF

        # telemetry (console only): frame id, detections, fps, jpeg size
        if frame_id % 10 == 0:
            print("[tele] fid=%d objs=%d %.1ffps jpeg=%dB" % (
                frame_id, len(detections), clock.fps(), len(jpeg_bytes)))


main()
