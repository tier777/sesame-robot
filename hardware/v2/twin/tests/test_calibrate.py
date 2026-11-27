from conftest import FIRMWARE
from twin import params as P
from twin.calibrate import HornFix, channel_table, horn_fix, main, safe_range

INO = FIRMWARE / "sesame-firmware-main.ino"


def test_horn_fix():
    assert horn_fix(170, 170) == HornFix(0, 0)
    assert horn_fix(170, 155) == HornFix(-1, 3)
    assert horn_fix(90, 134) == HornFix(2, 8)


def test_channel_table():
    t = channel_table(INO)
    assert t[0] == (0, "R1", 15) and t[5] == (5, "R3", 16) and len(t) == 8


def test_safe_range_covers_stand_and_rest():
    for j in P.JOINTS:
        lo, hi = safe_range(j)
        assert 0 <= lo <= hi <= 180
        assert lo <= min(P.CAD_POSE_FW[j], 90) and max(P.CAD_POSE_FW[j], 90) <= hi


def test_hip_cannot_swing_into_the_body():
    lo, hi = safe_range("R1")
    assert lo > 0      # R1 к 0° — бедро разворачивается назад вдоль корпуса и упирается


def test_pictures_and_table_written():
    main([])
    assert (P.OUT / "calib" / "R1_90.png").exists()
    assert "| 0 | R1 | 15 |" in (P.OUT / "calibration.md").read_text()


def test_cli_stop(capsys):
    main(["R1", "--stop", "155"])
    out = capsys.readouterr().out
    assert "servoSubtrim[0]" in out and "зуб" in out
