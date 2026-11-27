from twin.poser import export_cpp, format_command, mirror_allowed, open_mirror, round_pause


def test_format():
    assert format_command(5, 135, "main") == "5 135\n"
    assert format_command(5, 135, "tester") == "5,135\n"


def test_blocks_unsafe():
    safe = {"R1": (5, 155)}
    assert not mirror_allowed("R1", 170, safe)
    assert mirror_allowed("R1", 150, safe)
    assert mirror_allowed("R3", 10, safe)          # сустав без данных не блокируется


def test_export():
    src = export_cpp([([90] * 8, 300), ([135, 45, 45, 135, 0, 180, 0, 180], 500)], "hello")
    assert "inline void runHelloPose()" in src
    assert "setServoAngle(R1, 135);" in src and "delayWithFace(500);" in src
    assert src.count("setServoAngle(R1, 90);") == 1


def test_export_skips_unchanged():
    src = export_cpp([([90] * 8, 300), ([100] + [90] * 7, 200)], "nod")
    assert src.count("setServoAngle(") == 9


def test_pause_rounding():
    assert round_pause(0.333) == 350 and round_pause(0.01) == 50


def test_missing_port():
    assert open_mirror("/dev/cu.nope") is None
