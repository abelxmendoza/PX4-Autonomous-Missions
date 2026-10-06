"""MAVLink connection strings, in the style ground-station tools use.

    udpin:HOST:PORT     listen (PX4 SITL sends offboard MAVLink to :14540)
    udp:HOST:PORT       send to a remote endpoint
    serial:DEVICE[:BAUD] UART/USB (default 57600)
"""

from __future__ import annotations

from dataclasses import dataclass

from px4_offboard.comms.serial_transport import STANDARD_BAUDS

UDP_KINDS = ("udpin", "udp")


@dataclass(frozen=True)
class ConnectionSpec:
    kind: str
    host: str | None = None
    port: int | None = None
    device: str | None = None
    baud: int | None = None

    def __str__(self) -> str:
        if self.kind == "serial":
            return f"serial:{self.device}:{self.baud}"
        return f"{self.kind}:{self.host}:{self.port}"


def parse_connection(text: str) -> ConnectionSpec:
    parts = text.split(":")
    kind = parts[0]
    if kind in UDP_KINDS:
        if len(parts) != 3 or not parts[1]:
            raise ValueError(f"expected {kind}:HOST:PORT, got {text!r}")
        try:
            port = int(parts[2])
        except ValueError as exc:
            raise ValueError(f"bad port in {text!r}") from exc
        if not 0 < port < 65536:
            raise ValueError(f"port out of range in {text!r}")
        return ConnectionSpec(kind, host=parts[1], port=port)
    if kind == "serial":
        if len(parts) not in (2, 3) or not parts[1]:
            raise ValueError(f"expected serial:DEVICE[:BAUD], got {text!r}")
        baud = 57600
        if len(parts) == 3:
            try:
                baud = int(parts[2])
            except ValueError as exc:
                raise ValueError(f"bad baud in {text!r}") from exc
        if baud not in STANDARD_BAUDS:
            raise ValueError(f"{baud} is not a standard UART rate")
        return ConnectionSpec("serial", device=parts[1], baud=baud)
    raise ValueError(f"unsupported connection type in {text!r}")
