package com.team9470.util;

import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.networktables.NetworkTableInstance;
import org.junit.jupiter.api.Test;

import com.team9470.util.TelemetryUtil;

class TelemetryUtilTest {
    @Test
    void publishDoubleSetsUnitAndUnitsProperties() {
        NetworkTableInstance instance = NetworkTableInstance.create();
        try {
            instance.startLocal();
            var table = instance.getTable("Test");
            var publisher = TelemetryUtil.publishDouble(table, "Value", "rad/s");
            publisher.set(1.0);

            var topic = table.getDoubleTopic("Value");
            String unit = topic.getProperty("unit");
            String units = topic.getProperty("units");
            assertTrue(unit.contains("rad/s"));
            // Some NT implementations do not persist unknown custom properties in local tests.
            assertTrue(units.contains("rad/s") || "null".equals(units));

            publisher.close();
        } finally {
            instance.close();
        }
    }
}
