package com.team9470.subsystems.shooter;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

class ShooterInterpolationMapsTest {
    @Test
    void airTimeInterpolatesBetweenPoints() {
        assertEquals(0.975, ShooterInterpolationMaps.getAirTime(2.25), 1e-9);
    }

    @Test
    void airTimeUsesBoundaryValuesOutsideMapRange() {
        assertEquals(0.0, ShooterInterpolationMaps.getAirTime(-1.0), 1e-9);
        assertEquals(1.30, ShooterInterpolationMaps.getAirTime(10.0), 1e-9);
    }
}
