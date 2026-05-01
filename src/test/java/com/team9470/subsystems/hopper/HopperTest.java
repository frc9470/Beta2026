package com.team9470.subsystems.hopper;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

class HopperTest {
    @Test
    void scaledFeedVoltageUsesQuarterPower() {
        assertEquals(-3.0, Hopper.scaledFeedVoltage(0.25), 1e-9);
    }
}
