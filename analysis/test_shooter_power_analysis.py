import importlib.util
import tempfile
import unittest
from pathlib import Path

import pandas as pd


SCRIPT_PATH = Path(__file__).with_name("shooter_power_analysis.py")
SPEC = importlib.util.spec_from_file_location("shooter_power_analysis", SCRIPT_PATH)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(MODULE)


class ShooterPowerAnalysisTest(unittest.TestCase):
    def test_compute_step_response_metrics(self) -> None:
        data = pd.DataFrame(
            [
                {
                    "timestamp_sec": 0.00,
                    "run_id": 1,
                    "mode": "CLOSED_LOOP_STEP_SWEEP",
                    "segment_index": 0,
                    "commanded_rpm": 0.0,
                    "commanded_volts": 0.0,
                    "measured_rpm": 0.0,
                    "battery_volts": 12.0,
                    "flywheel_avg_motor_volts": 0.0,
                    "flywheel_total_supply_current_amps": 0.0,
                    "flywheel_avg_stator_current_amps": 0.0,
                    "electrical_power_watts": 0.0,
                    "velocity_error_rpm": 0.0,
                    "at_setpoint": False,
                    "settled": False,
                    "aborted": False,
                },
                {
                    "timestamp_sec": 1.00,
                    "run_id": 1,
                    "mode": "CLOSED_LOOP_STEP_SWEEP",
                    "segment_index": 1,
                    "commanded_rpm": 2000.0,
                    "commanded_volts": 0.0,
                    "measured_rpm": 1500.0,
                    "battery_volts": 12.0,
                    "flywheel_avg_motor_volts": 6.0,
                    "flywheel_total_supply_current_amps": 20.0,
                    "flywheel_avg_stator_current_amps": 30.0,
                    "electrical_power_watts": 240.0,
                    "velocity_error_rpm": 500.0,
                    "at_setpoint": False,
                    "settled": False,
                    "aborted": False,
                },
                {
                    "timestamp_sec": 1.20,
                    "run_id": 1,
                    "mode": "CLOSED_LOOP_STEP_SWEEP",
                    "segment_index": 1,
                    "commanded_rpm": 2000.0,
                    "commanded_volts": 0.0,
                    "measured_rpm": 1850.0,
                    "battery_volts": 12.0,
                    "flywheel_avg_motor_volts": 5.0,
                    "flywheel_total_supply_current_amps": 15.0,
                    "flywheel_avg_stator_current_amps": 20.0,
                    "electrical_power_watts": 180.0,
                    "velocity_error_rpm": 150.0,
                    "at_setpoint": False,
                    "settled": False,
                    "aborted": False,
                },
                {
                    "timestamp_sec": 1.40,
                    "run_id": 1,
                    "mode": "CLOSED_LOOP_STEP_SWEEP",
                    "segment_index": 1,
                    "commanded_rpm": 2000.0,
                    "commanded_volts": 0.0,
                    "measured_rpm": 2010.0,
                    "battery_volts": 12.0,
                    "flywheel_avg_motor_volts": 4.0,
                    "flywheel_total_supply_current_amps": 10.0,
                    "flywheel_avg_stator_current_amps": 15.0,
                    "electrical_power_watts": 120.0,
                    "velocity_error_rpm": -10.0,
                    "at_setpoint": True,
                    "settled": True,
                    "aborted": False,
                },
            ]
        )

        metrics = MODULE.compute_step_response_metrics(data)
        self.assertEqual(len(metrics), 1)
        row = metrics.iloc[0]
        self.assertAlmostEqual(row["idle_rpm"], 0.0)
        self.assertAlmostEqual(row["target_rpm"], 2000.0)
        self.assertAlmostEqual(row["time_to_90_sec"], 0.2, places=6)
        self.assertAlmostEqual(row["time_to_setpoint_sec"], 0.4, places=6)
        self.assertGreater(row["step_energy_j"], 0.0)

    def test_recommend_active_rpm_rejects_candidates_above_ready_cap(self) -> None:
        hold_summary = pd.DataFrame(
            [
                {"commanded_rpm": 0.0, "electrical_power_watts": 0.0},
                {"commanded_rpm": 1000.0, "electrical_power_watts": 60.0},
                {"commanded_rpm": 2000.0, "electrical_power_watts": 120.0},
            ]
        )
        step_metrics = pd.DataFrame(
            [
                {
                    "run_id": 1,
                    "segment_index": 1,
                    "idle_rpm": 0.0,
                    "target_rpm": 2000.0,
                    "time_to_90_sec": 0.35,
                    "time_to_setpoint_sec": 0.45,
                    "overshoot_rpm": 10.0,
                    "step_energy_j": 90.0,
                },
                {
                    "run_id": 1,
                    "segment_index": 3,
                    "idle_rpm": 1000.0,
                    "target_rpm": 2000.0,
                    "time_to_90_sec": 0.25,
                    "time_to_setpoint_sec": 0.55,
                    "overshoot_rpm": 5.0,
                    "step_energy_j": 60.0,
                },
            ]
        )
        recommendation = MODULE.recommend_active_rpm(
            step_metrics,
            hold_summary,
            {
                "ready_cap_sec": 0.5,
                "inactive_rpm": 0.0,
                "active_windows": [{"shot_rpm": 2000.0, "pre_shot_hold_sec": 0.2, "occurrences_per_match": 1}],
            },
        )
        self.assertEqual(recommendation["recommended_active_rpm"], 0.0)

    def test_run_analysis_writes_outputs(self) -> None:
        with tempfile.TemporaryDirectory() as tmp_dir:
            root = Path(tmp_dir)
            input_dir = root / "input"
            output_dir = root / "output"
            input_dir.mkdir()
            pd.DataFrame(
                [
                    {
                        "timestamp_sec": 0.0,
                        "run_id": 1,
                        "mode": "VELOCITY_HOLD_SWEEP",
                        "segment_index": 0,
                        "commanded_rpm": 1000.0,
                        "commanded_volts": 0.0,
                        "measured_rpm": 990.0,
                        "battery_volts": 12.0,
                        "flywheel_avg_motor_volts": 3.0,
                        "flywheel_total_supply_current_amps": 8.0,
                        "flywheel_avg_stator_current_amps": 10.0,
                        "electrical_power_watts": 96.0,
                        "velocity_error_rpm": 10.0,
                        "at_setpoint": True,
                        "settled": True,
                        "aborted": False,
                    }
                ]
            ).to_csv(input_dir / "hold.csv", index=False)
            (root / "demand.yaml").write_text(
                '{"ready_cap_sec": 0.5, "inactive_rpm": 0.0, "active_windows": []}\n',
                encoding="utf-8",
            )

            MODULE.run_analysis(input_dir, output_dir, root / "demand.yaml")
            self.assertTrue((output_dir / "hold_power_vs_rpm.png").exists())
            self.assertTrue((output_dir / "summary.md").exists())
            self.assertTrue((output_dir / "recommendation.json").exists())


if __name__ == "__main__":
    unittest.main()
