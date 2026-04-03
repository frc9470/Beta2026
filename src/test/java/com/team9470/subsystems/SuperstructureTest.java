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
}
