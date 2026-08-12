"""A real Modbus/TCP server exposing the cell, and a client to drive it.

The register semantics live in :mod:`cell_fieldbus.core.registers`; this module
is only the transport. pymodbus 3.x provides both ends, so the integration test
in ``tests/test_fieldbus.py`` starts this server on a loopback port and talks to
it with the actual client stack -- function codes, framing and all -- rather
than calling the state machine directly and asserting that Python works.

WHY BOTHER WITH A SOCKET IN A TEST

Because the errors that cost time on a real integration are not in the state
machine. They are: an address that is off by one because the PLC counts from
40001; a value that reads as 4464 instead of 70000 because nobody clamped it; a
32-bit value assembled in the wrong word order; and a client that polls faster
than the server updates and sees a torn pair of registers. Only the first is
visible without a transport, and the map is generated from one table precisely
so the first cannot happen.

The 32-bit convention here is high word first, which is what almost every PLC
library defaults to and what ``docs/fieldbus.md`` states. Nothing in the
current map is 32-bit; :func:`split_u32` and :func:`join_u32` exist because the
first thing anyone adds is a 32-bit part counter, and the word order is worth
deciding once.

EtherNet/IP is not implemented. The brief allows an open simulator, and a
faithful CIP implicit-messaging implementation is a project of its own; what is
here instead is a Modbus bridge that is real, plus
``docs/fieldbus.md#ethernet-ip`` describing exactly what the equivalent
assembly instances would carry. Claiming a protocol that is not implemented
would be the one thing worse.
"""

from __future__ import annotations

import asyncio
import logging
from dataclasses import dataclass
from typing import Callable, Dict, List, Optional

from cell_fieldbus.core.registers import (
    COIL_COUNT,
    COIL_NAMES,
    REGISTER_COUNT,
    CellStatus,
    FieldbusState,
)

LOGGER = logging.getLogger(__name__)

DEFAULT_PORT = 5020  # Not 502: binding below 1024 needs root, and this is a demo.
DEFAULT_UNIT_ID = 1


def split_u32(value: int) -> List[int]:
    """32-bit unsigned to two registers, high word first."""
    value = int(value) & 0xFFFFFFFF
    return [(value >> 16) & 0xFFFF, value & 0xFFFF]


def join_u32(words: List[int]) -> int:
    """Two registers, high word first, back to a 32-bit unsigned."""
    return ((int(words[0]) & 0xFFFF) << 16) | (int(words[1]) & 0xFFFF)


@dataclass
class BridgeConfig:
    host: str = "127.0.0.1"
    port: int = DEFAULT_PORT
    unit_id: int = DEFAULT_UNIT_ID
    scan_period: float = 0.02  # 20 ms, a typical PLC scan.


class ModbusBridge:
    """Serves the cell's status block and consumes the PLC's command coils.

    Usage is deliberately explicit rather than context-managed: a fieldbus
    bridge that silently stops when a block exits is a fieldbus bridge that
    stops during a cycle.
    """

    def __init__(self, config: Optional[BridgeConfig] = None, state: Optional[FieldbusState] = None) -> None:
        self.config = config or BridgeConfig()
        self.state = state or FieldbusState()
        self._context = None
        self._server_task: Optional[asyncio.Task] = None
        self._scan_task: Optional[asyncio.Task] = None
        self._on_start: Optional[Callable[[], None]] = None

    # -- lifecycle ---------------------------------------------------------

    def _build_context(self):
        from pymodbus.datastore import ModbusSequentialDataBlock, ModbusServerContext, ModbusSlaveContext

        slave = ModbusSlaveContext(
            co=ModbusSequentialDataBlock(0, [False] * COIL_COUNT),
            hr=ModbusSequentialDataBlock(0, self.state.status.to_registers()),
            di=ModbusSequentialDataBlock(0, [False] * COIL_COUNT),
            ir=ModbusSequentialDataBlock(0, [0] * REGISTER_COUNT),
        )
        return ModbusServerContext(slaves={self.config.unit_id: slave}, single=False)

    async def start(self, on_start: Optional[Callable[[], None]] = None) -> None:
        """Start the server and the scan loop."""
        from pymodbus.server import StartAsyncTcpServer

        self._on_start = on_start
        self._context = self._build_context()
        self._server_task = asyncio.create_task(
            StartAsyncTcpServer(context=self._context, address=(self.config.host, self.config.port))
        )
        await asyncio.sleep(0.15)  # Let the listener bind before anyone connects.
        self._scan_task = asyncio.create_task(self._scan_loop())

    async def stop(self) -> None:
        for task in (self._scan_task, self._server_task):
            if task is not None:
                task.cancel()
        for task in (self._scan_task, self._server_task):
            if task is not None:
                try:
                    await task
                except (asyncio.CancelledError, Exception):  # noqa: B902 - shutdown is best effort
                    pass
        self._scan_task = None
        self._server_task = None

    # -- scan --------------------------------------------------------------

    async def _scan_loop(self) -> None:
        while True:
            self.scan_once()
            await asyncio.sleep(self.config.scan_period)

    def scan_once(self) -> None:
        """One PLC-scan equivalent: read coils, step the state, write registers."""
        if self._context is None:
            return
        slave = self._context[self.config.unit_id]
        raw = slave.getValues(1, 0, count=COIL_COUNT)
        coils: Dict[int, bool] = {index: bool(raw[index]) for index in COIL_NAMES}
        self.state.update(coils)
        if self.state.take_start_request() and self._on_start is not None:
            self._on_start()
        slave.setValues(3, 0, self.state.status.to_registers())

    def publish(self, status: CellStatus) -> None:
        """Push a status block immediately, without waiting for the next scan."""
        self.state.status = status
        if self._context is not None:
            self._context[self.config.unit_id].setValues(3, 0, status.to_registers())


class ModbusCellClient:
    """The PLC's side: start cycles, read status, reset faults."""

    def __init__(self, config: Optional[BridgeConfig] = None) -> None:
        self.config = config or BridgeConfig()
        self._client = None

    async def connect(self) -> None:
        from pymodbus.client import AsyncModbusTcpClient

        self._client = AsyncModbusTcpClient(self.config.host, port=self.config.port)
        await self._client.connect()
        if not self._client.connected:
            raise ConnectionError(f"could not connect to {self.config.host}:{self.config.port}")

    async def close(self) -> None:
        if self._client is not None:
            self._client.close()
            self._client = None

    async def write_coil(self, address: int, value: bool) -> None:
        await self._call(self._client.write_coil, address, value)

    async def pulse_coil(self, address: int, hold: float = 0.06) -> None:
        """Set a coil, hold it past one scan, and clear it.

        Commands are edge-triggered, so the hold has to outlast a scan period
        or the rising edge is never observed. Getting this wrong produces the
        classic symptom: the cell starts sometimes.
        """
        await self.write_coil(address, True)
        await asyncio.sleep(hold)
        await self.write_coil(address, False)

    async def read_status(self) -> CellStatus:
        response = await self._call(self._client.read_holding_registers, 0, count=REGISTER_COUNT)
        return CellStatus.from_registers(list(response.registers))

    async def read_coils(self) -> List[bool]:
        response = await self._call(self._client.read_coils, 0, count=COIL_COUNT)
        return list(response.bits)[:COIL_COUNT]

    async def _call(self, method, *args, **kwargs):
        """Call a pymodbus method, raising on a protocol-level error.

        pymodbus returns exception *responses* rather than raising them, so a
        caller that does not check ``isError()`` treats an illegal-address
        response as a successful read of whatever happens to be in the object.
        """
        if self._client is None:
            raise ConnectionError("client is not connected")
        response = await method(*args, slave=self.config.unit_id, **kwargs)
        if response.isError():
            raise IOError(f"modbus error from {method.__name__}: {response}")
        return response
