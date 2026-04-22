package com.team9470.subsystems;

import static org.junit.jupiter.api.Assertions.assertFalse;
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
    void autoStageProbeRequiresCurrentAndSlowdown() {
        assertTrue(Superstructure.shouldRequestAutoStageFromProbe(14.0, 3.0, 12.0, 4.0));
        assertFalse(Superstructure.shouldRequestAutoStageFromProbe(10.0, 3.0, 12.0, 4.0));
        assertFalse(Superstructure.shouldRequestAutoStageFromProbe(14.0, 4.5, 12.0, 4.0));
    }
}
