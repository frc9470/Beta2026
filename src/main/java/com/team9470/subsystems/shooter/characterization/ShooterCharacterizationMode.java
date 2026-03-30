package com.team9470.subsystems.shooter.characterization;

public enum ShooterCharacterizationMode {
    DISABLED(0),
    VELOCITY_HOLD_SWEEP(1),
    OPEN_LOOP_VOLTAGE_SWEEP(2),
    CLOSED_LOOP_STEP_SWEEP(3);

    private final int code;

    ShooterCharacterizationMode(int code) {
        this.code = code;
    }

    public int code() {
        return code;
    }
}
