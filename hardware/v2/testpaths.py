"""Общие пути для тестов двойника и рамы v2."""
from pathlib import Path

V2 = Path(__file__).resolve().parent
REPO = V2.parents[1]
FIRMWARE = REPO / "firmware"
FIXTURES = V2 / "twin" / "tests" / "fixtures"
