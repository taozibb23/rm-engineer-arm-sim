"""占位机械臂协议（DRAFT —— 待电控/组长登记确认，非正式 RoboLink 分配）。

⚠ 重要：
- 现有 RoboLink 只有云台帧（0x20/0x23），**没有臂关节/夹爪帧**。这里的 msg_type 与载荷
  都是占位草案，接真实电控前必须与电控对账并重新登记编号。
- 角度域当前为 geometric_placeholder（几何角）。接真实电机前必须改电机角并标定传动比/零偏，
  否则违反"未确认禁止下发几何角冒充电机角"。
- 帧格式仿 RoboLink（头+类型+长度+载荷+CRC32），便于日后替换为真实 codec。
"""
from __future__ import annotations
import struct
import zlib
from dataclasses import dataclass

MAGIC = b"RL"
VERSION = 0x02
# 载荷布局：seq u32 | stamp_us u64 | psi f32 | alpha f32 | beta f32 | gripper u8 | flags/status u8
_PAYLOAD = struct.Struct("<IQfffBB")
PAYLOAD_LEN = _PAYLOAD.size  # 26


@dataclass
class ArmControl:
    seq: int
    stamp_us: int
    psi: float
    alpha: float
    beta: float
    gripper: int      # 0=张开 1=闭合
    flags: int = 0


@dataclass
class ArmFeedback:
    seq: int
    stamp_us: int
    psi: float
    alpha: float
    beta: float
    gripper: int
    status: int       # 0=ok 1=moving 2=fault


def _pack_frame(msg_type: int, payload: bytes) -> bytes:
    head = MAGIC + bytes([VERSION, msg_type]) + struct.pack("<H", len(payload))
    body = head + payload
    crc = zlib.crc32(body) & 0xFFFFFFFF
    return body + struct.pack("<I", crc)


def _unpack_frame(buf: bytes):
    if len(buf) < 10 or buf[:2] != MAGIC:
        raise ValueError("bad magic/short frame")
    ver, mt = buf[2], buf[3]
    (plen,) = struct.unpack("<H", buf[4:6])
    if len(buf) != 6 + plen + 4:
        raise ValueError("length mismatch")
    body = buf[:6 + plen]
    (crc,) = struct.unpack("<I", buf[6 + plen:])
    if zlib.crc32(body) & 0xFFFFFFFF != crc:
        raise ValueError("CRC mismatch")
    return mt, buf[6:6 + plen]


def encode_control(c: ArmControl, msg_type: int) -> bytes:
    return _pack_frame(msg_type, _PAYLOAD.pack(c.seq, c.stamp_us, c.psi, c.alpha, c.beta, c.gripper, c.flags))


def decode_control(buf: bytes, msg_type: int) -> ArmControl:
    mt, p = _unpack_frame(buf)
    if mt != msg_type:
        raise ValueError(f"unexpected msg_type {mt}")
    seq, ts, psi, a, b, g, fl = _PAYLOAD.unpack(p)
    return ArmControl(seq, ts, psi, a, b, g, fl)


def encode_feedback(f: ArmFeedback, msg_type: int) -> bytes:
    return _pack_frame(msg_type, _PAYLOAD.pack(f.seq, f.stamp_us, f.psi, f.alpha, f.beta, f.gripper, f.status))


def decode_feedback(buf: bytes, msg_type: int) -> ArmFeedback:
    mt, p = _unpack_frame(buf)
    if mt != msg_type:
        raise ValueError(f"unexpected msg_type {mt}")
    seq, ts, psi, a, b, g, st = _PAYLOAD.unpack(p)
    return ArmFeedback(seq, ts, psi, a, b, g, st)


def self_test():
    c = ArmControl(7, 123456, 0.1, 0.2, 0.3, 1, 0)
    buf = encode_control(c, 0x24)
    c2 = decode_control(buf, 0x24)
    assert (c2.seq, c2.gripper) == (7, 1) and abs(c2.alpha - 0.2) < 1e-6
    # CRC 篡改必须被拒
    bad = bytearray(buf); bad[8] ^= 0xFF
    try:
        decode_control(bytes(bad), 0x24); raise AssertionError("CRC should fail")
    except ValueError:
        pass
    return "PASS: 控制帧编解码 + CRC 篡改拒绝"


if __name__ == "__main__":
    print(self_test())
