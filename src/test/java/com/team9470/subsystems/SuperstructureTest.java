package com.team9470.subsystems;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

class SuperstructureTest {
    @Test
    void normalReleaseRequiresLiveFirePermission() {
        assertFalse(Superstructure.shouldAllowRelease(false, false, false, true));
        assertTrue(Superstructure.shouldAllowRelease(false, false, true, false));
    }

    @Test
    void timedShotUsesWindowPermitBeforeReleaseStarts() {
        assertFalse(Superstructure.shouldAllowRelease(true, false, false, false));
        assertTrue(Superstructure.shouldAllowRelease(true, false, false, true));
    }

    @Test
    void timedShotFallsBackToLiveFireChecksAfterReleaseStarts() {
        assertFalse(Superstructure.shouldAllowRelease(true, true, false, true));
        assertTrue(Superstructure.shouldAllowRelease(true, true, true, false));
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
    void alignmentLatchRequiresInitialTightAcquire() {
        assertFalse(Superstructure.shouldTreatAlignmentAsReady(false, false, Math.toRadians(10.0)));
        assertTrue(Superstructure.shouldTreatAlignmentAsReady(true, false, Math.toRadians(0.0)));
    }

    @Test
    void alignmentLatchAllowsWiderContinuousWindow() {
        assertTrue(Superstructure.shouldTreatAlignmentAsReady(false, true, Math.toRadians(10.0)));
        assertFalse(Superstructure.shouldTreatAlignmentAsReady(false, true, Math.toRadians(16.0)));
    }
}
