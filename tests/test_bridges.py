"""Basic unit tests — run without hardware via: python -m pytest tests/"""

import sys, os, math, time
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import pytest
from src.vio.base import Pose
from src.bridges.msp_bridge import _msp_v1_encode, _crc8_dvb_s2, _msp_v2_encode
from src.bridges.betaflight_bridge import PositionPID, _angle_to_rc


# ── MSP protocol tests ──────────────────────────────────────────────────────────

def test_msp_v1_header():
    pkt = _msp_v1_encode(201, b"\x01\x02\x03")
    assert pkt[:3] == b"$M<"
    assert pkt[3] == 3    # payload length
    assert pkt[4] == 201  # command


def test_msp_v1_checksum_empty():
    pkt = _msp_v1_encode(100, b"")
    # checksum = 0 XOR 100 XOR nothing = 100
    assert pkt[-1] == 100


def test_msp_v1_checksum_payload():
    pkt = _msp_v1_encode(201, bytes([0x01, 0x02]))
    # checksum = 2 ^ 201 ^ 1 ^ 2
    expected = 2 ^ 201 ^ 1 ^ 2
    assert pkt[-1] == expected


def test_msp_v2_header():
    pkt = _msp_v2_encode(0x1234, b"\xAB\xCD")
    assert pkt[:3] == b"$X<"


def test_crc8_known():
    # Empty input → 0
    assert _crc8_dvb_s2(b"") == 0


# ── Position PID tests ──────────────────────────────────────────────────────────

def test_pid_no_error():
    pid = PositionPID(kp=1.0, ki=0.0)
    pid.set_target(0.0, 0.0)
    roll, pitch = pid.update(0.0, 0.0, 0.0, 0.1)
    assert roll == pytest.approx(0.0, abs=1e-6)
    assert pitch == pytest.approx(0.0, abs=1e-6)


def test_pid_north_error():
    pid = PositionPID(kp=1.0, ki=0.0, kd=0.0)
    pid.set_target(1.0, 0.0)  # 1 m north
    roll, pitch = pid.update(0.0, 0.0, 0.0, 0.1)
    # With zero yaw: north error → pitch forward (negative pitch body)
    assert pitch < 0


def test_rc_centre():
    assert _angle_to_rc(0.0) == 1500


def test_rc_max():
    assert _angle_to_rc(math.radians(20), math.radians(20)) == 2000


def test_rc_min():
    assert _angle_to_rc(-math.radians(20), math.radians(20)) == 1000


# ── Pose covariance test ────────────────────────────────────────────────────────

def test_pose_covariance_length():
    p = Pose(quality=1.0)
    cov = p.to_covariance()
    assert len(cov) == 21


def test_pose_covariance_diagonal_nonzero():
    p = Pose(quality=1.0)
    cov = p.to_covariance(pos_noise=0.1)
    assert cov[0] > 0   # xx
    assert cov[6] > 0   # yy
    assert cov[11] > 0  # zz


# ── MockVO smoke test ───────────────────────────────────────────────────────────

def test_mock_vo_produces_pose():
    from src.vio.simple_vo import MockVO
    from src.vio.base import CameraFrame
    vo = MockVO()
    vo.initialize()
    pose = vo.update(CameraFrame(), [])
    assert pose is not None
    assert isinstance(pose.x, float)
    assert -100 < pose.x < 100
