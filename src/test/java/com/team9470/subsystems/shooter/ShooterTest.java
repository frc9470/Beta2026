package com.team9470.subsystems.shooter;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

class ShooterTest {
    @Test
    void closedLoopFlywheelCommandStopsWhenHoming() {
        assertEquals(0.0, Shooter.computeClosedLoopFlywheelCommandRps(true, 42.0, true), 1e-9);
    }

    @Test
    void closedLoopFlywheelCommandUsesRequestedSpeed() {
        assertEquals(42.0, Shooter.computeClosedLoopFlywheelCommandRps(false, 42.0, false), 1e-9);
    }

    @Test
    void closedLoopFlywheelCommandFallsBackToSimpleIdleBehavior() {
        assertEquals(2000.0 / 60.0, Shooter.computeClosedLoopFlywheelCommandRps(false, 0.0, false), 1e-9);
        assertEquals(2550.0 / 60.0, Shooter.computeClosedLoopFlywheelCommandRps(false, 0.0, true), 1e-9);
    }
}
