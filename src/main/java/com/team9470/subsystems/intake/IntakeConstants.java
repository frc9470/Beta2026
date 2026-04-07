package com.team9470.subsystems.intake;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;

public class IntakeConstants {
    // NOTE: CAN IDs are in Ports.java (INTAKE_PIVOT, INTAKE_ROLLER)

    // Physical Constants
    public static final double kPivotGearRatio = 20.0;

    // Setpoints
    public static final Angle kDeployAngle = Degrees.of(4.0); // Down/Floor
    public static final Angle kDeployHighAngle = Degrees.of(-10.0); // Deploy +10 deg
    public static final Angle kAgitateMiddleAngle = Degrees.of(100.0); // Upper compression target during shot agitation
    public static final Angle kRetractAngle = Degrees.of(110.0); // Up/Stowed
    public static final double kRollerVoltage = 9.6;
    public static final boolean kUseLegacyAgitationOscillation = false;
    public static final double kAgitateCompressDurationSec = 1.25;
    public static final double kAgitateFrequencyHz = 1.0; // Legacy deploy/mid oscillation frequency

    // Simulation
    public static final double kIntakeLength = 0.3; // meters
    public static final double kIntakeMass = 4.0; // kg

    // Current Limits
    public static final double kPivotStatorCurrentLimit = 80.0;
    public static final double kPivotSupplyCurrentLimit = 30.0;
    public static final double kRollerStatorCurrentLimit = 30.0;

    // ==================== HOMING ====================
    // Intake homes to hardstop at retract position (90 degrees up)
    public static final double kHomingVoltage = 3.0; // Positive = toward retract/up
    public static final double kStallCurrentThreshold = 20.0; // Amps
    public static final double kStallTimeThreshold = 0.1; // Seconds at stall
    public static final double kStallVelocityThreshold = 0.05; // Mechanism rot/s (~18 deg/s)
    public static final Angle kHomePosition = Degrees.of(136.0); // Angle at hardstop

    // Motor Configs
    public static final TalonFXConfiguration kPivotConfig = new TalonFXConfiguration();
    public static final TalonFXConfiguration kLeftRollerConfig = new TalonFXConfiguration();
    public static final TalonFXConfiguration kRightRollerConfig = new TalonFXConfiguration();

    // -------------------- Unit / Gear Conversion Helpers --------------------
    public static double pivotAngleToMechanismRotations(Angle angle) {
        return angle.in(Rotations);
    }

    public static Angle pivotMechanismRotationsToAngle(double mechanismRotations) {
        return Rotations.of(mechanismRotations);
    }

    public static double pivotMechanismRotationsToMotorRotations(double pivotMechanismRotations) {
        return pivotMechanismRotations * kPivotGearRatio;
    }

    public static double pivotMotorRotationsToMechanismRotations(double pivotMotorRotations) {
        return pivotMotorRotations / kPivotGearRatio;
    }

    static {
        // Pivot Config
        kPivotConfig.Slot0.kP = 30.0;
        kPivotConfig.Slot0.kI = 0.0;
        kPivotConfig.Slot0.kD = 0.0;
        kPivotConfig.Slot0.kV = 0.5;
        kPivotConfig.Slot0.kA = 0.1;
        kPivotConfig.Slot0.kG = 0.5; // Gravity hold
        kPivotConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        kPivotConfig.Feedback.SensorToMechanismRatio = kPivotGearRatio;
        kPivotConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        kPivotConfig.MotionMagic.MotionMagicCruiseVelocity = 4.0; // rad/s
        kPivotConfig.MotionMagic.MotionMagicAcceleration = 7.0; // rad/s^2
        kPivotConfig.MotionMagic.MotionMagicJerk = 0;
        kPivotConfig.CurrentLimits
                .withStatorCurrentLimit(kPivotStatorCurrentLimit)
                .withSupplyCurrentLimit(kPivotSupplyCurrentLimit)
                .withStatorCurrentLimitEnable(true)
                .withSupplyCurrentLimitEnable(false);

        // Roller Config
        kLeftRollerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        kLeftRollerConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        kLeftRollerConfig.CurrentLimits
                .withSupplyCurrentLimitEnable(false)
                .withStatorCurrentLimit(kRollerStatorCurrentLimit)
                .withStatorCurrentLimitEnable(true);

        kRightRollerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        kRightRollerConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        kRightRollerConfig.CurrentLimits
                .withSupplyCurrentLimitEnable(false)
                .withStatorCurrentLimit(kRollerStatorCurrentLimit)
                .withStatorCurrentLimitEnable(true);
    }
}
