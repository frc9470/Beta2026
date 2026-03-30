package com.team9470.subsystems.shooter.characterization;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

class ShooterCharacterizationSessionTest {
    @Test
    void velocitySweepTransitionsAcrossSegments() {
        ShooterCharacterizationConfig config = new ShooterCharacterizationConfig(
                new double[] {0.0, 250.0},
                1.0,
                0.25,
                new double[] {2.0},
                0.5,
                new double[] {0.0},
                new double[] {1000.0},
                1.0,
                0.02);
        ShooterCharacterizationSession session = new ShooterCharacterizationSession(
                1,
                ShooterCharacterizationMode.VELOCITY_HOLD_SWEEP,
                config,
                0.0);

        ShooterCharacterizationSession.Sample first = session.update(0.1, false);
        assertTrue(first.active());
        assertEquals(0, first.segmentIndex());
        assertFalse(first.settled());

        ShooterCharacterizationSession.Sample second = session.update(1.1, true);
        assertTrue(second.active());
        assertEquals(1, second.segmentIndex());
        assertEquals(250.0, second.commandedRpm(), 1e-9);

        ShooterCharacterizationSession.Sample finalSample = session.update(3.2, true);
        assertTrue(finalSample.complete());
        assertFalse(finalSample.active());
    }

    @Test
    void stepSweepAlternatesIdleAndTargetSegments() {
        ShooterCharacterizationConfig config = new ShooterCharacterizationConfig(
                new double[] {0.0},
                1.0,
                0.25,
                new double[] {2.0},
                0.5,
                new double[] {0.0, 500.0},
                new double[] {2000.0},
                1.5,
                0.02);
        ShooterCharacterizationSession session = new ShooterCharacterizationSession(
                2,
                ShooterCharacterizationMode.CLOSED_LOOP_STEP_SWEEP,
                config,
                0.0);

        ShooterCharacterizationSession.Sample idle = session.update(0.1, true);
        assertEquals(0.0, idle.commandedRpm(), 1e-9);

        ShooterCharacterizationSession.Sample step = session.update(1.1, false);
        assertEquals(2000.0, step.commandedRpm(), 1e-9);

        ShooterCharacterizationSession.Sample nextIdle = session.update(2.7, true);
        assertEquals(500.0, nextIdle.commandedRpm(), 1e-9);
    }
}
