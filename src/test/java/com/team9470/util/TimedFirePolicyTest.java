package com.team9470.util;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Rotation2d;
import org.junit.jupiter.api.Test;

class TimedFirePolicyTest {
    private static AutoAim.ShootingSolution solution(double naiveAirTimeSec) {
        return new AutoAim.ShootingSolution(
                new Rotation2d(),
                30.0,
                3200.0,
                0.0,
                4.0,
                4.0,
                naiveAirTimeSec,
                0.0,
                0.0,
                true);
    }

    @Test
    void feedModeFallsBackToNormalBehavior() {
        var decision = TimedFirePolicy.evaluate(
                true,
                true,
                false,
                0.1,
                true,
                true,
                true,
                true,
                true,
                true,
                solution(0.2));

        assertFalse(decision.timedAutoArmCandidate());
        assertFalse(decision.allowFeed());
    }

    @Test
    void timingUnknownBlocksTimedRelease() {
        var decision = TimedFirePolicy.evaluate(
                true,
                false,
                false,
                0.1,
                true,
                true,
                true,
                true,
                true,
                false,
                solution(0.2));

        assertTrue(decision.timedAutoArmCandidate());
        assertFalse(decision.allowFeed());
    }

    @Test
    void topSensorMustBeBlockedForTimedRelease() {
        var decision = TimedFirePolicy.evaluate(
                true,
                true,
                false,
                0.1,
                true,
                false,
                true,
                true,
                true,
                false,
                solution(0.2));

        assertTrue(decision.timedAutoArmCandidate());
        assertFalse(decision.allowFeed());
    }

    @Test
    void inactiveHoldOpensReleaseWindow() {
        var decision = TimedFirePolicy.evaluate(
                true,
                true,
                false,
                0.1,
                true,
                true,
                true,
                true,
                true,
                false,
                solution(0.2));

        assertTrue(decision.timedAutoArmCandidate());
        assertTrue(decision.allowFeed());
    }

    @Test
    void activeAtPressUsesNormalImmediateBehavior() {
        var decision = TimedFirePolicy.evaluate(
                true,
                true,
                true,
                0.0,
                false,
                true,
                true,
                true,
                true,
                false,
                solution(0.2));

        assertFalse(decision.timedAutoArmCandidate());
        assertFalse(decision.allowFeed());
    }
}
