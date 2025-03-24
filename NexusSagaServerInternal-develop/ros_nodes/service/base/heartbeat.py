from dataclasses import dataclass


@dataclass
class Heartbeat:
    status: str  # for now, there is no literal/enum type hints yet
