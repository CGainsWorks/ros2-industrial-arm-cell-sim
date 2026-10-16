#!/usr/bin/env python3
"""Run the cell as an EtherNet/IP adapter: OpENer in front, the simulated cell behind.

    python3 scripts/enip_cell.py --opener /opt/enip/OpENer --interface eth0

Starts the OpENer adapter built with ``cell_fieldbus/enip/adapter`` as a child
process, and the cell side of the adapter (:class:`cell_fieldbus.core.enip.EnipCell`)
in this one. A PLC, or any EtherNet/IP scanner, can then open a Class 1
connection to assemblies 150/100 on port 44818 and run the cell.

Cycles are the offline pick and place cycle from ``cell_bringup.core.cycle``,
planned in full when the PLC starts one and then played out segment by
segment at ``--time-scale`` (1.0 is real time), so the PLC sees the state move
through the cycle and a light curtain break lands in a real segment.

This is the simulated cell, not a robot. ``docker/enip-compose.yml`` runs it
next to a scanner, which is what CI does.
"""

from __future__ import annotations

import argparse
import asyncio
import logging
import os
import signal
import subprocess
import sys
import tempfile
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
for _package in ("cell_common", "cell_motion", "cell_grasping", "cell_calibration", "cell_inspection",
                 "cell_bringup", "cell_fieldbus", "cell_safety"):
    sys.path.insert(0, str(REPO_ROOT / "src" / _package))

import numpy as np  # noqa: E402

from cell_bringup.core.cycle import CycleRunner  # noqa: E402
from cell_common.cell_model import load_cell, sample_part_pose  # noqa: E402
from cell_fieldbus.core.enip import EnipCell, EnipCellConfig  # noqa: E402


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--opener", type=Path, help="Path to the OpENer binary; omit to run the cell side only")
    parser.add_argument("--interface", default="eth0", help="Network interface OpENer binds to")
    parser.add_argument("--time-scale", type=float, default=1.0)
    parser.add_argument("--seed", type=int, default=20260101)
    parser.add_argument("--link-port", type=int, default=EnipCellConfig.link_port)
    parser.add_argument("--adapter-port", type=int, default=EnipCellConfig.adapter_port)
    arguments = parser.parse_args()
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(name)s %(message)s")

    cell = load_cell()
    runner = CycleRunner(cell)
    names = [part.name for part in cell.parts]

    def run_cycle(index: int):
        rng = np.random.default_rng(arguments.seed + index)
        pose = sample_part_pose(cell, rng)
        result = runner.run(pose, rng)
        logging.info("cycle %d: %s %s in %.2f s (simulated)", index, pose["part"],
                     "ok" if result.success else f"{result.stage}/{result.failure}", result.cycle_time)
        return result, names.index(pose["part"]) + 1

    config = EnipCellConfig(
        link_port=arguments.link_port, adapter_port=arguments.adapter_port, time_scale=arguments.time_scale
    )
    enip_cell = EnipCell(config, run_cycle)

    adapter = None
    if arguments.opener is not None:
        environment = dict(os.environ, CELL_LINK_PORT=str(config.link_port), CELL_ADAPTER_PORT=str(config.adapter_port))
        # OpENer keeps its non-volatile data in files in its working
        # directory. A fresh one per run keeps a previous run's settings out.
        workdir = tempfile.mkdtemp(prefix="opener-")
        adapter = subprocess.Popen([str(arguments.opener), arguments.interface], env=environment, cwd=workdir)
        logging.info("OpENer adapter started on %s (pid %d)", arguments.interface, adapter.pid)

    async def serve() -> None:
        await enip_cell.start()
        stop = asyncio.Event()
        loop = asyncio.get_running_loop()
        for signum in (signal.SIGINT, signal.SIGTERM):
            try:
                loop.add_signal_handler(signum, stop.set)
            except NotImplementedError:  # pragma: no cover - Windows
                pass
        while not stop.is_set():
            if adapter is not None and adapter.poll() is not None:
                logging.error("OpENer exited with %s", adapter.returncode)
                break
            try:
                await asyncio.wait_for(stop.wait(), timeout=0.5)
            except asyncio.TimeoutError:
                pass
        await enip_cell.stop()

    try:
        asyncio.run(serve())
    finally:
        if adapter is not None and adapter.poll() is None:
            adapter.send_signal(signal.SIGINT)
            try:
                adapter.wait(timeout=5)
            except subprocess.TimeoutExpired:
                adapter.kill()
    # OpENer exits with 200 plus the signal number when it is stopped by one.
    clean = (None, 0, -signal.SIGINT, 200 + signal.SIGINT)
    return 0 if adapter is None or adapter.returncode in clean else 1


if __name__ == "__main__":
    raise SystemExit(main())
