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


class FakePort:
    def __init__(self, fail=False):
        self.out, self.fail, self.closed = [], fail, False

    def write(self, b):
        if self.fail:
            raise OSError("кабель выдернут")
        self.out.append(b.decode())

    def close(self):
        self.closed = True


def test_mirror_rate_limited_and_latest_only():
    from twin.poser import Mirror
    port = FakePort()
    m = Mirror(port, "main")
    for k, fw in enumerate(range(100, 160)):          # быстрый рывок ползунка за 0,12 с
        m.update(0, fw)
        m.flush(now=k * 0.002)
    assert len(port.out) <= 3                            # не чаще одной команды в 50 мс
    m.flush(now=1.0)
    assert port.out[-1] == "0 159\n"                     # дошло последнее значение, без промежуточных


def test_mirror_write_failure_disables_not_crashes():
    from twin.poser import Mirror
    m = Mirror(FakePort(fail=True), "main")
    m.update(0, 120)
    m.flush(now=1.0)
    assert not m.alive


def test_serial_opened_without_reset_lines():
    from twin.poser import open_mirror
    seen = {}

    class FakeSerial:
        def __init__(self):
            self.port = None

        def open(self):
            seen.update(dtr=self.dtr, rts=self.rts, port=self.port)

    m = open_mirror("/dev/cu.test", factory=FakeSerial)
    assert m is not None and seen == {"dtr": False, "rts": False, "port": "/dev/cu.test"}


def test_export_behaves_like_firmware_poses():
    src = export_cpp([([90] * 8, 300)], "hello")
    assert src.index("runStandPose(0);") < src.index("setServoAngle(")
    assert 'if (currentCommand == "hello") currentCommand = "";' in src
