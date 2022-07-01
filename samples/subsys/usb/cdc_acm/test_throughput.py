#!/usr/bin/env python3
#
# Copyright (c) 2022 Johann Fischer
#
# SPDX-License-Identifier: Apache-2.0
#

import argparse
import math
import sys
import time
from contextlib import suppress

import serial
import tqdm

# Ring buffer size used by the sample, see samples/subsys/usb/cdc_acm/src/main.c.
# Larger payloads make the device throttle OUT before we start to drain IN.
SAMPLE_RING_BUF_SIZE = 1024

# Time the device may take to echo one payload.
ECHO_TIMEOUT = 2.0

# Writes block on flow control, so allow about one round trip.
WRITE_TIMEOUT = 1.0

# Time to wait for leftovers while resyncing.
RESYNC_TIMEOUT = 0.2

# Bytes shown on both sides of a mismatch.
DUMP_MARGIN = 8

# Time the sample may take to assert DCD and DSR.
LINE_STATE_TIMEOUT = 1.0

# Alternated to force a SET_LINE_CODING request, the host suppresses it if the
# line coding does not change.
BAUDRATES = (115200, 921600)

KIB = 1024


def positive_int(value):
    number = int(value)

    if number <= 0:
        raise argparse.ArgumentTypeError(f"{value} is not a positive number")

    return number


def wait_line_state(ser_dev, timeout):
    """Wait for DCD and DSR, notified by the sample."""
    deadline = time.perf_counter() + timeout

    while time.perf_counter() < deadline:
        if ser_dev.cd and ser_dev.dsr:
            return True

        time.sleep(0.01)

    return False


def make_payload(pattern, length, iteration):
    """Rotate the pattern so that a stale buffer does not compare equal."""
    shift = iteration % len(pattern)
    rotated = pattern[shift:] + pattern[:shift]
    repeats = math.ceil(length / len(rotated))

    return (rotated * repeats)[:length]


def control_request(ser_dev, idx):
    """Issue one class request, to be called while a transfer is in flight."""
    rtype = idx % 3

    if rtype == 0:
        # SET_LINE_CODING
        ser_dev.baudrate = BAUDRATES[idx // 3 % 2]
    elif rtype == 1:
        # SET_CONTROL_LINE_STATE, RTS bit
        ser_dev.rts = not ser_dev.rts
    else:
        # SET_CONTROL_LINE_STATE, DTR bit
        ser_dev.dtr = not ser_dev.dtr


def resync(ser_dev):
    """Discard what the device still echoes after a failed iteration."""
    timeout = ser_dev.timeout
    ser_dev.timeout = RESYNC_TIMEOUT

    with suppress(serial.SerialException):
        while ser_dev.read(SAMPLE_RING_BUF_SIZE):
            pass

    ser_dev.timeout = timeout
    ser_dev.reset_input_buffer()


def first_difference(expected, actual):
    pairs = zip(expected, actual, strict=False)

    for offset, (out_byte, in_byte) in enumerate(pairs):
        if out_byte != in_byte:
            return offset

    return min(len(expected), len(actual))


def report_mismatch(iteration, expected, actual):
    offset = first_difference(expected, actual)
    begin = max(offset - DUMP_MARGIN, 0)
    end = offset + DUMP_MARGIN

    print(f"Corrupted data in iteration {iteration}, offset {offset}")
    print(f"  expected: {expected[begin:end].hex(' ')}")
    print(f"  received: {actual[begin:end].hex(' ')}")


def report_short_read(iteration, expected, actual):
    if not actual:
        print(f"Timeout in iteration {iteration}, no data received")
    else:
        print(f"Short read in iteration {iteration}, "
              f"expected {len(expected)} bytes, got {len(actual)}")


def parse_args():
    parser = argparse.ArgumentParser(description="Test CDC ACM serial sample.")
    parser.add_argument("-p", "--port", default="/dev/ttyACM1",
                        help="Serial device path (default: /dev/ttyACM1)")
    parser.add_argument("-l", "--len", type=positive_int, default=1024,
                        help="Payload length in bytes (default: 1024)")
    parser.add_argument("-i", "--iterations", type=positive_int, default=2000,
                        help="Number of iterations (default: 2000)")
    parser.add_argument("-c", "--control-interval", type=int, default=0,
                        help="Class request every N iterations (default: off)")

    return parser.parse_args()


def main():
    args = parse_args()

    if args.len > SAMPLE_RING_BUF_SIZE:
        print(f"Warning: payload exceeds ring buffer size {SAMPLE_RING_BUF_SIZE}, "
              "writes may time out")

    try:
        ser_dev = serial.Serial(args.port, 115200, timeout=ECHO_TIMEOUT,
                                write_timeout=WRITE_TIMEOUT)
    except serial.SerialException as e:
        print(f"Error opening serial port {args.port}: {e}")
        sys.exit(1)

    if not wait_line_state(ser_dev, LINE_STATE_TIMEOUT):
        print("Warning: DCD/DSR not set, no SerialState notification received")

    pattern = bytes(range(256))

    ser_dev.reset_input_buffer()
    ser_dev.reset_output_buffer()

    total_sent = 0
    total_received = 0
    completed = 0
    errors = 0
    control_requests = 0
    failed = False

    pbar = tqdm.tqdm(total=args.iterations * args.len,
                     desc="Progress",
                     dynamic_ncols=True,
                     unit="B",
                     unit_scale=True,
                     unit_divisor=KIB)

    loop_start = time.perf_counter()

    try:
        for iteration in range(args.iterations):
            data_out = make_payload(pattern, args.len, iteration)

            # --- write ---
            try:
                ser_dev.write(data_out)
            except serial.SerialException as e:
                print(f"Write error in iteration {iteration}: {e}")
                # A partial write is echoed back and would desynchronize
                # the following iterations.
                resync(ser_dev)
                errors += 1
                continue

            total_sent += len(data_out)

            # --- class request while the echo is in flight ---
            if args.control_interval and iteration % args.control_interval == 0:
                try:
                    control_request(ser_dev, control_requests)
                    control_requests += 1
                except (serial.SerialException, OSError) as e:
                    print(f"Control error in iteration {iteration}: {e}")
                    errors += 1

            # --- read ---
            try:
                # Returns less than requested if the timeout expires.
                data_in = ser_dev.read(args.len)
            except serial.SerialException as e:
                print(f"Read error in iteration {iteration}: {e}")
                resync(ser_dev)
                errors += 1
                continue

            total_received += len(data_in)

            if len(data_in) != len(data_out):
                report_short_read(iteration, data_out, data_in)
                failed = True
                break

            if data_in != data_out:
                report_mismatch(iteration, data_out, data_in)
                failed = True
                break

            completed += 1
            pbar.update(args.len)

    except KeyboardInterrupt:
        print("\nInterrupted")

    finally:
        pbar.close()
        ser_dev.close()

    elapsed = time.perf_counter() - loop_start

    # Writes block on flow control, so only the rate over both directions is
    # meaningful here.
    roundtrip_rate = 0.0

    if elapsed > 0:
        roundtrip_rate = (total_sent + total_received) / KIB / elapsed

    print("\n--- Final statistics ---")
    print(f"Iterations completed:  {completed}/{args.iterations}")
    print(f"Errors:                {errors}")
    print(f"Control requests:      {control_requests}")
    print(f"Total bytes sent:      {total_sent}")
    print(f"Total bytes received:  {total_received}")
    print(f"Round trip rate:       {roundtrip_rate:.2f} KiB/s")

    if failed or errors or completed != args.iterations:
        sys.exit(1)


if __name__ == "__main__":
    main()
