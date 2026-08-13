#!/usr/bin/env python3
import json
import subprocess
import sys
import tempfile
from pathlib import Path

import numpy as np
from PIL import Image


def write_pair(root: Path, frame: int, left: int, right: int) -> None:
    for eye, value in (("L", left), ("R", right)):
        rgb = np.full((96, 128, 3), value, dtype=np.uint8)
        rgb[25:65, 30 + frame : 60 + frame] = min(255, value + 25)
        Image.fromarray(rgb).save(root / f"frame{frame}_color_{eye}.bmp")


with tempfile.TemporaryDirectory(prefix="bvr-flicker-test-") as temp:
    root = Path(temp)
    clean = root / "clean"
    flicker = root / "flicker"
    ui_structural = root / "ui_structural"
    clean.mkdir(); flicker.mkdir(); ui_structural.mkdir()
    for frame in range(1, 7):
        write_pair(clean, frame, 90 + frame, 92 + frame)
        write_pair(flicker, frame, 90 + frame, 255 if frame == 4 else 92 + frame)
        write_pair(ui_structural, frame, 90 + frame, 92 + frame)
    (ui_structural / "incident.json").write_text(json.dumps({
        "schemaVersion": 1,
        "captureSource": "openxr-simulator-ui-quad",
        "triggerFrame": 4,
        "reason": "UI_NOT_RECOMPOSED_AFTER_PROJECTION",
    }), encoding="utf-8")

    analyzer = Path(__file__).parents[1] / "analyze_openxr_flicker.py"
    for corpus, expected in ((clean, "NO_FLICKER_DETECTED"), (flicker, "FLICKER_DETECTED"),
                             (ui_structural, "FLICKER_DETECTED")):
        output = corpus / "report"
        subprocess.run([sys.executable, str(analyzer), str(corpus), "--output", str(output)], check=True)
        result = json.loads((output / "metrics.json").read_text(encoding="utf-8"))
        assert result["verdict"] == expected, (corpus, result)
        if corpus == ui_structural:
            assert result["captureSource"] == "openxr-simulator-ui-quad"
            assert any(item.get("reason") == "UI_NOT_RECOMPOSED_AFTER_PROJECTION"
                       for item in result["incidents"])
        assert (output / "contact_sheet.png").is_file()
        assert (output / "differences.png").is_file()
        assert (output / "motion_compensated.png").is_file()
        assert (output / "LLM_REVIEW.md").is_file()
print("flicker analyzer regression: PASS")
