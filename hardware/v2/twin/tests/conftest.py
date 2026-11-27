import sys
from pathlib import Path

V2 = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(V2))
REPO = V2.parents[1]
FIRMWARE = REPO / "firmware"
FIXTURES = Path(__file__).parent / "fixtures"
