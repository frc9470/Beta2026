package com.team9470.subsystems.shooter.characterization;

import static org.junit.jupiter.api.Assertions.assertEquals;

import com.team9470.telemetry.structs.ShooterCharacterizationSnapshot;
import org.junit.jupiter.api.Test;

class ShooterCharacterizationCsvLoggerTest {
    @Test
    void formatsCsvRowInExpectedColumnOrder() {
        ShooterCharacterizationSnapshot snapshot = new ShooterCharacterizationSnapshot(
                1.0,
                2,
                3,
                4,
                3000.0,
                6.0,
                2950.0,
                11.9,
                5.2,
                42.0,
                55.0,
                499.8,
                50.0,
                true,
                false,
                false);

        assertEquals(
                "1.0,2,CLOSED_LOOP_STEP_SWEEP,4,3000.0,6.0,2950.0,11.9,5.2,42.0,55.0,499.8,50.0,true,false,false",
                ShooterCharacterizationCsvLogger.toCsvRow(
                        ShooterCharacterizationMode.CLOSED_LOOP_STEP_SWEEP,
                        snapshot));
    }
}
