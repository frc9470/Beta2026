package com.team9470.telemetry;

import static org.junit.jupiter.api.Assertions.assertEquals;

import com.team9470.telemetry.structs.AutoAimSolverSnapshot;
import com.team9470.telemetry.structs.DriveStatusSnapshot;
import com.team9470.telemetry.structs.HopperPreloadSnapshot;
import com.team9470.telemetry.structs.HopperSnapshot;
import com.team9470.telemetry.structs.IntakeSnapshot;
import com.team9470.telemetry.structs.PracticeTimerSnapshot;
import com.team9470.telemetry.structs.ShooterCharacterizationSnapshot;
import com.team9470.telemetry.structs.ShooterSnapshot;
import com.team9470.telemetry.structs.SimSnapshot;
import com.team9470.telemetry.structs.SuperstructureSnapshot;
import com.team9470.telemetry.structs.TimedShotSnapshot;
import com.team9470.telemetry.structs.VisionCameraSnapshot;
import com.team9470.telemetry.structs.VisionSnapshot;
import com.team9470.telemetry.structs.YShotSnapshot;
import edu.wpi.first.util.struct.Struct;
import java.nio.ByteBuffer;
import org.junit.jupiter.api.Test;

class SnapshotStructsTest {
    private static <T> void assertRoundTrip(Struct<T> struct, T value) {
        ByteBuffer buffer = ByteBuffer.allocate(struct.getSize());
        struct.pack(buffer, value);
        buffer.flip();
        assertEquals(value, struct.unpack(buffer));
    }

    @Test
    void driveStatusRoundTrip() {
        assertRoundTrip(DriveStatusSnapshot.struct, new DriveStatusSnapshot(1.2, 250.0, 0.004));
    }

    @Test
    void intakeRoundTrip() {
        assertRoundTrip(IntakeSnapshot.struct,
                new IntakeSnapshot(false, true, false, 2, 1.0, 1.1, 1.05, 0.05, 2.0, 12.0, 4.5, 6.0, 8.0));
    }

    @Test
    void shooterRoundTrip() {
        assertRoundTrip(ShooterSnapshot.struct,
                new ShooterSnapshot(
                        false,
                        true,
                        true,
                        3,
                        0.5,
                        0.5,
                        0.49,
                        0.01,
                        1.2,
                        10.0,
                        9.0,
                        4.0,
                        80.0,
                        79.0,
                        1.0,
                        6.0,
                        40.0,
                        45.0));
    }

    @Test
    void shooterCharacterizationRoundTrip() {
        assertRoundTrip(ShooterCharacterizationSnapshot.struct,
                new ShooterCharacterizationSnapshot(
                        12.5,
                        7,
                        2,
                        4,
                        3000.0,
                        6.0,
                        2850.0,
                        11.8,
                        5.5,
                        60.0,
                        75.0,
                        708.0,
                        150.0,
                        true,
                        false,
                        false));
    }

    @Test
    void hopperRoundTrip() {
        assertRoundTrip(HopperSnapshot.struct, new HopperSnapshot(true, 10.5, 9.0, 25.0, 30.0, true, false));
    }

    @Test
    void hopperPreloadRoundTrip() {
        assertRoundTrip(HopperPreloadSnapshot.struct, new HopperPreloadSnapshot(true, 3, 2));
    }

    @Test
    void superstructureRoundTrip() {
        assertRoundTrip(SuperstructureSnapshot.struct, new SuperstructureSnapshot(true, false, 2.5, 0.04));
    }

    @Test
    void visionRoundTrip() {
        assertRoundTrip(VisionSnapshot.struct, new VisionSnapshot(42, true, false, 2, 2, 0));
    }

    @Test
    void visionCameraRoundTrip() {
        assertRoundTrip(VisionCameraSnapshot.struct, new VisionCameraSnapshot(true, 99, 3, 2, 0.12, 123.45));
    }

    @Test
    void simRoundTrip() {
        assertRoundTrip(SimSnapshot.struct, new SimSnapshot(80.0, 0.7, 5, 2.5));
    }

    @Test
    void autoAimRoundTrip() {
        assertRoundTrip(AutoAimSolverSnapshot.struct, new AutoAimSolverSnapshot(1, true, 4.2, 6.1, true, 0.4, 52.0));
    }

    @Test
    void yShotRoundTrip() {
        assertRoundTrip(YShotSnapshot.struct, new YShotSnapshot(true, 50.0, 0.6, 48.0, 0.58, 3.2, true));
    }

    @Test
    void timedShotRoundTrip() {
        assertRoundTrip(TimedShotSnapshot.struct, new TimedShotSnapshot(true, true, false, true, false, 1.25, 0.42, false, 2, false));
    }

    @Test
    void practiceTimerRoundTrip() {
        assertRoundTrip(PracticeTimerSnapshot.struct, new PracticeTimerSnapshot(
                4,
                true,
                false,
                true,
                true,
                3,
                200.0,
                120.0,
                185.0,
                10.0,
                125.0,
                29.0,
                125.0,
                125.0,
                0,
                2,
                true,
                true,
                false,
                20.0,
                10.0,
                125.0));
    }
}
