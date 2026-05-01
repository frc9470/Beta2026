package com.team9470.subsystems;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

class SuperstructureTest {
    @Test
    void releaseRequiresCanFire() {
        assertFalse(Superstructure.shouldAllowRelease(false, false));
        assertTrue(Superstructure.shouldAllowRelease(true, false));
    }

    @Test
    void releaseStaysOnceStarted() {
        // Even if canFire drops, release stays latched
        assertTrue(Superstructure.shouldAllowRelease(false, true));
        assertTrue(Superstructure.shouldAllowRelease(true, true));
    }

    @Test
    void flywheelLatchDoesNotRelaxFirstShot() {
        assertFalse(Superstructure.shouldTreatFlywheelAsReady(false, false, true));
        assertTrue(Superstructure.shouldTreatFlywheelAsReady(true, false, false));
    }

    @Test
    void flywheelLatchKeepsReleaseReadyAfterShotStarts() {
        assertTrue(Superstructure.shouldTreatFlywheelAsReady(false, true, true));
        assertFalse(Superstructure.shouldTreatFlywheelAsReady(false, true, false));
    }

    @Test
    void feedModeUsesWiderHeadingTolerance() {
        double normalToleranceRad = Math.toRadians(2.5);

        assertEquals(normalToleranceRad, Superstructure.getAlignmentToleranceRadForShot(false, normalToleranceRad));
        assertEquals(Math.toRadians(10.0), Superstructure.getAlignmentToleranceRadForShot(true, normalToleranceRad));
    }

    @Test
    void autoStageProbeRequiresCurrentAndSlowdown() {
        assertTrue(Superstructure.shouldRequestAutoStageFromProbe(14.0, 3.0, 12.0, 4.0));
        assertFalse(Superstructure.shouldRequestAutoStageFromProbe(10.0, 3.0, 12.0, 4.0));
        assertFalse(Superstructure.shouldRequestAutoStageFromProbe(14.0, 4.5, 12.0, 4.0));
    }

    @Test
    void overBumpFeedVoltageScaleDropsAboveThreeThousandRpm() {
        assertEquals(1.0, Superstructure.getOverBumpFeedVoltageScale(3000.0), 1e-9);
        assertEquals(0.25, Superstructure.getOverBumpFeedVoltageScale(3000.1), 1e-9);
    }

    @Test
    void feedModeUsesRelaxedFlywheelTolerance() {
        double normalToleranceRps = 1.0;

        assertEquals(normalToleranceRps, Superstructure.getFlywheelToleranceRpsForShot(false, normalToleranceRps),
                1e-9);
        assertEquals(150.0 / 60.0, Superstructure.getFlywheelToleranceRpsForShot(true, normalToleranceRps), 1e-9);
        assertFalse(Superstructure.isFlywheelAtSetpointForShot(false, 2.0, normalToleranceRps));
        assertTrue(Superstructure.isFlywheelAtSetpointForShot(true, 2.0, normalToleranceRps));
    }
}
