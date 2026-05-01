package com.team9470.subsystems.shooter;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

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

    @Test
    void overBumpFeedUsesThreeMeterShiftedFeedMap() {
        var oneMeterOverBump = ShooterInterpolationMaps.getOverBumpFeed(1.0);
        var fourMeterFeed = ShooterInterpolationMaps.getFeed(4.0);
        var sixMeterOverBump = ShooterInterpolationMaps.getOverBumpFeed(6.0);
        var nineMeterFeed = ShooterInterpolationMaps.getFeed(9.0);

        assertTrue(oneMeterOverBump.isPresent());
        assertTrue(fourMeterFeed.isPresent());
        assertTrue(sixMeterOverBump.isPresent());
        assertTrue(nineMeterFeed.isPresent());
        assertEquals(fourMeterFeed.get().hoodCommandDeg(), oneMeterOverBump.get().hoodCommandDeg(), 1e-9);
        assertEquals(fourMeterFeed.get().flywheelRpm(), oneMeterOverBump.get().flywheelRpm(), 1e-9);
        assertEquals(nineMeterFeed.get().hoodCommandDeg(), sixMeterOverBump.get().hoodCommandDeg(), 1e-9);
        assertEquals(nineMeterFeed.get().flywheelRpm(), sixMeterOverBump.get().flywheelRpm(), 1e-9);
    }
}
