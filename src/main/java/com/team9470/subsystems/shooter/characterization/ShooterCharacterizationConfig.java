package com.team9470.subsystems.shooter.characterization;

import com.team9470.subsystems.shooter.ShooterConstants;
import java.util.Arrays;

public record ShooterCharacterizationConfig(
        double[] rpmPlateaus,
        double plateauDwellSec,
        double plateauDiscardSec,
        double[] voltageSteps,
        double voltageStepHoldSec,
        double[] idleCandidatesRpm,
        double[] shotTargetsRpm,
        double stepTimeoutSec,
        double samplePeriodSec) {
    private static final double kPlateauStepRpm = 500.0;
    private static final double kVelocityPlateauDwellSec = 2.0;
    private static final double kVelocityPlateauDiscardSec = 0.75;
    private static final double kVoltageStepHoldSec = 0.75;
    private static final double kStepTimeoutSec = 2.0;
    private static final double kSamplePeriodSec = 0.02;

    public ShooterCharacterizationConfig {
        rpmPlateaus = rpmPlateaus.clone();
        voltageSteps = voltageSteps.clone();
        idleCandidatesRpm = idleCandidatesRpm.clone();
        shotTargetsRpm = shotTargetsRpm.clone();
    }

    @Override
    public double[] rpmPlateaus() {
        return rpmPlateaus.clone();
    }

    @Override
    public double[] voltageSteps() {
        return voltageSteps.clone();
    }

    @Override
    public double[] idleCandidatesRpm() {
        return idleCandidatesRpm.clone();
    }

    @Override
    public double[] shotTargetsRpm() {
        return shotTargetsRpm.clone();
    }

    public static ShooterCharacterizationConfig defaults() {
        double maxRpm = Math.min(4000.0, defaultMaxFlywheelRpm());
        return new ShooterCharacterizationConfig(
                rpmRange(0.0, maxRpm, kPlateauStepRpm),
                kVelocityPlateauDwellSec,
                kVelocityPlateauDiscardSec,
                new double[] {3.0, 6.0, 9.0},
                kVoltageStepHoldSec,
                filterByMaxRpm(new double[] {0.0, 1000.0, 2000.0, 3000.0}, maxRpm),
                defaultShotTargets(maxRpm),
                kStepTimeoutSec,
                kSamplePeriodSec);
    }

    public static double defaultMaxFlywheelRpm() {
        double maxMechanismRps = ShooterConstants.kMaxMotorSpeed / ShooterConstants.kFlywheelGearRatio;
        return Math.floor((maxMechanismRps * 60.0) / kPlateauStepRpm) * kPlateauStepRpm;
    }

    private static double[] defaultShotTargets(double maxRpm) {
        return Arrays.stream(new double[] {2500.0, 3500.0, 4000.0})
                .filter(rpm -> rpm <= maxRpm)
                .toArray();
    }

    private static double[] filterByMaxRpm(double[] values, double maxRpm) {
        return Arrays.stream(values)
                .filter(rpm -> rpm <= maxRpm)
                .toArray();
    }

    private static double[] rpmRange(double startRpm, double endRpm, double stepRpm) {
        int count = (int) Math.floor((endRpm - startRpm) / stepRpm) + 1;
        double[] values = new double[Math.max(1, count)];
        for (int i = 0; i < values.length; i++) {
            values[i] = startRpm + (i * stepRpm);
        }
        return values;
    }
}
