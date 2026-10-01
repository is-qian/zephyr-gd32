#!/usr/bin/env python3
"""Bus-side check for the GD32 power-board firmware (Dynamixel protocol 2.0).

The board is half-duplex on a single DATA wire. To talk to it with a plain
USB-UART (3.3 V levels): UART_TX -- 1K resistor -- DATA, UART_RX -- DATA,
GND -- GND. The adapter hears its own transmission, so every command first
reads back and discards its echo.

Usage:
    python3 dxl_probe.py /dev/ttyUSB0            # ping + read everything
    python3 dxl_probe.py /dev/ttyUSB0 --id 200
"""

import argparse
import struct
import sys

import serial

HDR = b"\xff\xff\xfd\x00"


def crc16(data):
    crc = 0
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x8005) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def add_stuffing(data):
    out = bytearray()
    run = 0
    for b in data:
        out.append(b)
        if run < 2 and b == 0xFF:
            run += 1
        elif run == 2 and b == 0xFF:
            run = 2
        elif run == 2 and b == 0xFD:
            out.append(0xFD)
            run = 0
        elif b == 0xFF:
            run = 1
        else:
            run = 0
    return bytes(out)


def remove_stuffing(data):
    out = bytearray()
    run = 0
    for b in data:
        if run == 3:
            run = 0
            if b == 0xFD:
                continue
        if run < 2 and b == 0xFF:
            run += 1
        elif run == 2 and b == 0xFF:
            run = 2
        elif run == 2 and b == 0xFD:
            run = 3
        elif b == 0xFF:
            run = 1
        else:
            run = 0
        out.append(b)
    return bytes(out)


def instruction(dev_id, instr, params=b""):
    params = add_stuffing(params)
    length = len(params) + 3
    pkt = HDR + bytes([dev_id]) + struct.pack("<H", length) + bytes([instr]) + params
    return pkt + struct.pack("<H", crc16(pkt))


def read_status(ser, timeout=0.2):
    ser.timeout = timeout
    hdr = ser.read(7)
    if len(hdr) < 7 or hdr[:4] != HDR:
        return None
    length = struct.unpack("<H", hdr[5:7])[0]
    body = ser.read(length)
    if len(body) < length:
        return None
    pkt = hdr + body
    if crc16(pkt[:-2]) != struct.unpack("<H", pkt[-2:])[0]:
        return None
    if body[0] != 0x55:
        return None
    return hdr[4], body[1], remove_stuffing(body[2:-2])


def transact(ser, pkt):
    ser.reset_input_buffer()
    ser.write(pkt)
    ser.flush()
    echo = ser.read(len(pkt))  # half-duplex: drop own echo
    if echo != pkt:
        print("warning: echo mismatch (wiring?)")
    return read_status(ser)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("port")
    ap.add_argument("--id", type=int, default=200)
    ap.add_argument("--baud", type=int, default=1_000_000)
    args = ap.parse_args()

    ser = serial.Serial(args.port, args.baud, timeout=0.1)

    r = transact(ser, instruction(args.id, 0x01))
    if r is None:
        print("PING: no answer")
        sys.exit(1)
    model, fw = struct.unpack("<HB", r[2])
    print(f"PING id={r[0]} err={r[1]} model={model} fw={fw}")

    # Single reads: one register at a time.
    names = {124: "gyro_x", 126: "gyro_y", 128: "gyro_z",
             130: "quat_x", 132: "quat_y", 134: "quat_z",
             136: "accel_x", 138: "accel_y", 140: "accel_z",
             142: "sample_counter", 143: "status",
             144: "voltage_0.1V", 146: "temperature_C"}
    sizes = {142: 1, 143: 1, 146: 1}
    for addr, name in names.items():
        size = sizes.get(addr, 2)
        r = transact(ser, instruction(args.id, 0x02, struct.pack("<HH", addr, size)))
        if r is None:
            print(f"  {name}: no answer")
            continue
        v = int.from_bytes(r[2], "little")
        print(f"  {name}: {v} (err={r[1]})")

    # One block read with everything.
    r = transact(ser, instruction(args.id, 0x02, struct.pack("<HH", 124, 23)))
    if r is None or len(r[2]) != 23:
        print("BLOCK READ 124..146: failed")
        sys.exit(1)
    d = r[2]
    gyro = struct.unpack_from("<3h", d, 0)
    quat = struct.unpack_from("<3H", d, 6)
    accel = struct.unpack_from("<3h", d, 12)
    print(f"BLOCK gyro={gyro} quat_f16={[hex(q) for q in quat]} accel={accel}")
    print(f"BLOCK counter={d[18]} status=0x{d[19]:02x} "
          f"voltage={struct.unpack_from('<H', d, 20)[0] / 10:.1f}V temp={d[22]}C")

    ser.close()


if __name__ == "__main__":
    main()
