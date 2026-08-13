from __future__ import annotations

import importlib.util
from pathlib import Path
import unittest


MODULE_PATH = Path(__file__).resolve().parents[1] / "capture_stereo_oracle.py"
SPEC = importlib.util.spec_from_file_location("capture_stereo_oracle", MODULE_PATH)
assert SPEC and SPEC.loader
ORACLE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(ORACLE)


def camera_sample(eye: str, marker: int, translation: float) -> dict[str, object]:
    view = [
        1.0, 0.0, 0.0, translation,
        0.0, 1.0, 0.0, 0.0,
        0.0, 0.0, 1.0, 0.0,
        0.0, 0.0, 0.0, 1.0,
    ]
    projection = [1.0 if row == column else 0.0 for row in range(4) for column in range(4)]
    inverse_view = view.copy()
    inverse_view[3] = -translation
    values = view[:12] + view + projection + inverse_view[:12] + [0.0] * (584 - 56)
    return {
        "eye": eye,
        "eyeMarkerSequence": marker,
        "drawPairKey": "0x1234",
        "address": "0x01000000" if eye == "left" else "0x02000000",
        "floatLE": values,
    }


class StereoOracleCameraTests(unittest.TestCase):
    def test_decodes_paired_camera_matrices(self) -> None:
        result = ORACLE.analyze_camera_uniform(
            {"samples": [camera_sample("left", 10, -0.03), camera_sample("right", 11, 0.03)]}
        )
        self.assertTrue(result["valid"], result)
        self.assertEqual(result["wordCount"], 584)
        self.assertGreater(result["differingWordCount"], 0)
        self.assertLess(result["matrixRelationships"]["viewTimesInvViewResidual"], 1.0e-6)

    def test_rejects_unpaired_camera_samples(self) -> None:
        left = camera_sample("left", 10, -0.03)
        right = camera_sample("right", 11, 0.03)
        right["drawPairKey"] = "0x9999"
        result = ORACLE.analyze_camera_uniform({"samples": [left, right]})
        self.assertFalse(result["valid"])


if __name__ == "__main__":
    unittest.main()
