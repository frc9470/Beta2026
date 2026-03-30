package com.team9470.subsystems.shooter.characterization;

public record ShooterCharacterizationStatus(
        boolean active,
        boolean complete,
        boolean aborted,
        int runId,
        ShooterCharacterizationMode mode,
        int segmentIndex,
        int totalSegments,
        double commandedRpm,
        double commandedVolts,
        String abortReason) {
    public static ShooterCharacterizationStatus idle() {
        return new ShooterCharacterizationStatus(
                false,
                false,
                false,
                0,
                ShooterCharacterizationMode.DISABLED,
                -1,
                0,
                0.0,
                0.0,
                "");
    }
}
