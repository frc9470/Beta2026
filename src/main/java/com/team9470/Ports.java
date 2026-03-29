package com.team9470;

import com.team254.lib.drivers.CanDeviceId;

public class Ports {
    // CANivore bus name
    public static final String CANIVORE = "canivore";

    // ==================== SHOOTER ====================
    public static final CanDeviceId FLYWHEEL_1 = new CanDeviceId(1, CANIVORE);
    public static final CanDeviceId FLYWHEEL_2 = new CanDeviceId(2, CANIVORE);
    public static final CanDeviceId FLYWHEEL_3 = new CanDeviceId(3, CANIVORE);
    public static final CanDeviceId FLYWHEEL_4 = new CanDeviceId(4, CANIVORE);
    public static final CanDeviceId HOOD_MOTOR = new CanDeviceId(7, CANIVORE);

    // ==================== HOPPER ====================
    public static final CanDeviceId FEEDER_RIGHT = new CanDeviceId(5, CANIVORE);
    public static final CanDeviceId FEEDER_LEFT = new CanDeviceId(6, CANIVORE);
    public static final CanDeviceId HOPPER_MOTOR = new CanDeviceId(8, CANIVORE);

    // ==================== INTAKE (RIO bus) ====================
    public static final CanDeviceId INTAKE_PIVOT = new CanDeviceId(9, CANIVORE);
    public static final CanDeviceId INTAKE_ROLLER_RIGHT = new CanDeviceId(10, CANIVORE);
    public static final CanDeviceId INTAKE_ROLLER_LEFT = new CanDeviceId(13, CANIVORE);
}
