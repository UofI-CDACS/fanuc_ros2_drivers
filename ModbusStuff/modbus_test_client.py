"""
Modbus TCP Test Client
----------------------
Connects to the test server (default localhost:5020) and loops forever,
writing random values to every coil and holding register, reading them
back, and printing PASS/FAIL per register each iteration.

Press Ctrl-C to stop.  All registers are reset to 0 on exit.

Usage:
    python3 modbus_test_client.py [--host localhost] [--port 5020] [--interval 1.0]
"""

import argparse
import random
import sys
import time

from pymodbus.client import ModbusTcpClient

# ── Register map (must match the server) ─────────────────────────────────────
COIL_REGISTER_MAP = {
    "DJ_Has_Dice":      0,
    "Bill_Has_Dice":    1,
    "Ready_For_Pickup": 2,
    "Cycle_Active":     3,
}

HOLDING_REGISTER_MAP = {
    "Total_Pip_Count": 0,
    "Total_Retries":   1,
    "Bill_Retries":    2,
    "DJ_Retries":      3,
    "Last_Known_Pip":  4,
}

SLAVE = 1
PASS  = "\033[32mPASS\033[0m"
FAIL  = "\033[31mFAIL\033[0m"


def run_iteration(client: ModbusTcpClient, iteration: int) -> int:
    """Write random values to all registers, read back, report. Returns fail count."""
    failures = 0

    print(f"\n{'─' * 55}")
    print(f"  Iteration {iteration}")
    print(f"{'─' * 55}")

    # ── Coils (random bool) ───────────────────────────────────────────────────
    print("  Coils:")
    for name, addr in sorted(COIL_REGISTER_MAP.items(), key=lambda x: x[1]):
        write_val = random.choice([True, False])

        rw = client.write_coil(addr, write_val, slave=SLAVE)
        if rw.isError():
            print(f"    {FAIL}  [{addr}] {name:<20}  write error: {rw}")
            failures += 1
            continue

        rr = client.read_coils(addr, count=1, slave=SLAVE)
        if rr.isError():
            print(f"    {FAIL}  [{addr}] {name:<20}  read error: {rr}")
            failures += 1
            continue

        read_val = bool(rr.bits[0])
        ok  = (read_val == write_val)
        tag = PASS if ok else FAIL
        print(f"    {tag}  [{addr}] {name:<20}  wrote={write_val!s:5}  read={read_val!s:5}")
        if not ok:
            failures += 1

    # ── Holding registers (random int 0–255) ──────────────────────────────────
    print("  Holding registers:")
    for name, addr in sorted(HOLDING_REGISTER_MAP.items(), key=lambda x: x[1]):
        write_val = random.randint(0, 255)

        rw = client.write_register(addr, write_val, slave=SLAVE)
        if rw.isError():
            print(f"    {FAIL}  [{addr}] {name:<20}  write error: {rw}")
            failures += 1
            continue

        rr = client.read_holding_registers(addr, count=1, slave=SLAVE)
        if rr.isError():
            print(f"    {FAIL}  [{addr}] {name:<20}  read error: {rr}")
            failures += 1
            continue

        read_val = rr.registers[0]
        ok  = (read_val == write_val)
        tag = PASS if ok else FAIL
        print(f"    {tag}  [{addr}] {name:<20}  wrote={write_val:3}  read={read_val:3}")
        if not ok:
            failures += 1

    status = "All PASS" if failures == 0 else f"{failures} FAILED"
    colour = "\033[32m" if failures == 0 else "\033[31m"
    print(f"\n  {colour}{status}\033[0m")
    return failures


def reset_all(client: ModbusTcpClient) -> None:
    for addr in COIL_REGISTER_MAP.values():
        client.write_coil(addr, False, slave=SLAVE)
    for addr in HOLDING_REGISTER_MAP.values():
        client.write_register(addr, 0, slave=SLAVE)


def main(host: str, port: int, interval: float) -> None:
    print(f"\nConnecting to Modbus server at {host}:{port} ...")
    client = ModbusTcpClient(host, port=port)

    if not client.connect():
        print(f"\n{FAIL}  Could not connect to {host}:{port}")
        print("  Is modbus_test_server.py running?")
        sys.exit(1)

    print(f"  Connected (slave={SLAVE})  —  looping every {interval}s, Ctrl-C to stop")

    iteration   = 1
    total_iters = 0
    total_fails = 0

    try:
        while True:
            fails = run_iteration(client, iteration)
            total_fails += fails
            total_iters += 1
            iteration   += 1
            time.sleep(interval)
    except KeyboardInterrupt:
        print(f"\n\nStopped after {total_iters} iteration(s).  "
              f"Total failures: {total_fails}")
        print("Resetting all registers to 0 ...")
        reset_all(client)
        client.close()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Modbus TCP looping test client")
    parser.add_argument("--host",     default="localhost",
                        help="Server hostname or IP (default: localhost)")
    parser.add_argument("--port",     type=int,   default=5020,
                        help="Server port (default: 5020)")
    parser.add_argument("--interval", type=float, default=1.0,
                        help="Seconds between iterations (default: 1.0)")
    args = parser.parse_args()
    main(args.host, args.port, args.interval)
