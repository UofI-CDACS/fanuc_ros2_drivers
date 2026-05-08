"""
Modbus TCP Test Server
----------------------
Runs a local Modbus TCP server on PORT (default 5020) using the
register map from ClaudeAssignmentModbusRegister.py.

Initialises all coils and holding registers to 0, then prints their
names/addresses so you can verify the client is hitting the right spots.

Usage:
    python3 modbus_test_server.py [--port 5020]
"""

import asyncio
import argparse
import contextlib
import logging
import sys

from pymodbus.datastore import (
    ModbusSequentialDataBlock,
    ModbusSlaveContext,
    ModbusServerContext,
)
from pymodbus.server import StartAsyncTcpServer

# ── Register map (must match what the client uses) ───────────────────────────
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

NUM_COILS    = max(COIL_REGISTER_MAP.values()) + 1      # 4
NUM_HOLDING  = max(HOLDING_REGISTER_MAP.values()) + 1   # 5
STATUS_INTERVAL = 1.0

# ── Logging ───────────────────────────────────────────────────────────────────
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s  %(levelname)-7s  %(message)s",
    datefmt="%H:%M:%S",
)
log = logging.getLogger(__name__)

RESET = "\033[0m"
BOLD = "\033[1m"
DIM = "\033[2m"
CYAN = "\033[36m"
GREEN = "\033[32m"
YELLOW = "\033[33m"


def _fmt_bool(value: bool) -> str:
    return f"{GREEN}ON {RESET}" if value else f"{DIM}off{RESET}"


def _build_table(title: str, items: dict[str, int], values: list[int], *, kind: str) -> list[str]:
    lines = [f"{BOLD}{title}{RESET}", f"{DIM}{'addr':>5}  {'name':<20}  value{RESET}"]
    for name, addr in sorted(items.items(), key=lambda item: item[1]):
        raw_value = values[addr] if addr < len(values) else None
        if kind == "coil":
            display_value = _fmt_bool(bool(raw_value)) if raw_value is not None else f"{YELLOW}n/a{RESET}"
        else:
            display_value = f"{CYAN}{raw_value:>5}{RESET}" if raw_value is not None else f"{YELLOW}n/a{RESET}"
        lines.append(f"[{addr:>2}]  {name:<20}  {display_value}")
    return lines


async def status_task(context: ModbusServerContext, interval: float, port: int) -> None:
    """Continuously render the current coil and holding-register values."""
    while True:
        slave = context[1]
        coil_values = slave.getValues(1, 0, count=NUM_COILS)
        holding_values = slave.getValues(3, 0, count=NUM_HOLDING)

        panel = [
            "\033[2J\033[H",
            f"{BOLD}{'=' * 66}{RESET}",
            f"{BOLD}  Modbus TCP Test Server{RESET}  {DIM}(slave=1){RESET}",
            f"{BOLD}{'=' * 66}{RESET}",
            f"Port: {CYAN}{port}{RESET}",
            "",
        ]
        panel.extend(_build_table("Coils", COIL_REGISTER_MAP, coil_values, kind="coil"))
        panel.append("")
        panel.extend(_build_table("Holding Registers", HOLDING_REGISTER_MAP, holding_values, kind="holding"))
        panel.append("")
        panel.append(f"{DIM}Refreshing every {interval:.1f}s  |  Ctrl-C to stop{RESET}")

        sys.stdout.write("\n".join(panel) + "\n")
        sys.stdout.flush()
        await asyncio.sleep(interval)


def build_context() -> ModbusServerContext:
    """Build a single-slave context with zeroed coils and holding registers.

    zero_mode=True makes pymodbus use 0-based addressing (address N in the
    Modbus PDU maps directly to index N in the data block).  Without it,
    pymodbus subtracts 1 from every address internally, which causes the
    last register in each block to be unreachable.
    """
    slave = ModbusSlaveContext(
        co=ModbusSequentialDataBlock(0, [False] * NUM_COILS),
        hr=ModbusSequentialDataBlock(0, [0]     * NUM_HOLDING),
        di=ModbusSequentialDataBlock(0, [False]),
        ir=ModbusSequentialDataBlock(0, [0]),
        zero_mode=True,
    )
    return ModbusServerContext(slaves={1: slave}, single=False)


async def main(port: int) -> None:
    context = build_context()

    log.info("=" * 55)
    log.info("  Modbus TCP Test Server — listening on 0.0.0.0:%d", port)
    log.info("=" * 55)
    log.info("Updating live status display every %.1fs", STATUS_INTERVAL)

    status = asyncio.create_task(status_task(context, STATUS_INTERVAL, port))
    try:
        await StartAsyncTcpServer(
            context=context,
            address=("0.0.0.0", port),
        )
    finally:
        status.cancel()
        with contextlib.suppress(asyncio.CancelledError):
            await status
        

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Modbus TCP test server")
    parser.add_argument("--port", type=int, default=5020,
                        help="TCP port to listen on (default: 5020)")
    args = parser.parse_args()

    try:
        asyncio.run(main(args.port))
    except KeyboardInterrupt:
        log.info("Server stopped.")
