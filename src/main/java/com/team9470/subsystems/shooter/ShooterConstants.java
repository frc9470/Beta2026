package com.team9470.subsystems.shooter;

import static edu.wpi.first.units.Units.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.MomentOfInertia;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class ShooterConstants {
        // NOTE: CAN IDs are in Ports.java (FLYWHEEL_1, FLYWHEEL_2,
        // FLYWHEEL_3, FLYWHEEL_4, HOOD_MOTOR)

        // Physical Constants
        public static final double kFlywheelGearRatio = 1.5;
        public static final double kFlywheelEfficiency = 0.6;
        public static final double kMaxMotorSpeed = 100.0; // RPS (~6000 RPM Kraken)

        // 9 lb*in^2 = 9 * 0.0002926397 ~= 0.002634 kg*m^2
        public static final MomentOfInertia kFlywheelMOI = KilogramSquareMeters.of(0.002634);

        public static final double kHoodGearRatio = 38.0; // 2:1 stage × 9.5:1 (10t pinion → 95t gear)
        public static final MomentOfInertia kHoodMOI = KilogramSquareMeters.of(0.05);
        public static final Distance kHoodLength = Meters.of(0.2);
        public static final Mass kHoodMass = Kilograms.of(2.0);

        // Current Limits
        public static final double kFlywheelStatorCurrentLimit = 40.0;
        public static final double kHoodSupplyCurrentLimit = 25.0;

        // Hood launch-angle limits (projectile pitch from horizontal).
        public static final Angle kMinHoodAngle = Degrees.of(15.0); // flattest shot
        public static final Angle kMaxHoodAngle = Degrees.of(45.0); // steepest shot

        // Field Geometry
        public static final Distance kShooterOffsetX = Inches.of(6.0); // Forward from robot center
        public static final Distance kShooterOffsetZ = Inches.of(19.0); // Up from center

        // ==================== HOMING ====================
        // Hood homes to hardstop at minimum launch angle.
        public static final double kHoodHomingVoltage = -2.0; // Toward min launch angle
        public static final double kHoodStallCurrentThreshold = 20.0; // Amps
        public static final double kHoodStallTimeThreshold = 0.1; // Seconds at stall
        public static final Angle kHoodHomePosition = Degrees.of(14.0); // Launch angle at hardstop

        // Motor Configs
        public static final TalonFXConfiguration kFlywheelConfig = new TalonFXConfiguration();
        public static final TalonFXConfiguration kFlywheelInvertedConfig = new TalonFXConfiguration();
        public static final TalonFXConfiguration kHoodConfig = new TalonFXConfiguration();

        // -------------------- Unit / Gear Conversion Helpers --------------------
        // Keep all gear-ratio math here to avoid duplicated scaling in subsystems.

        /** Convert launch angle (radians) to mechanism rotations. */
        public static double launchRadToMechanismRotations(double launchRad) {
                return Units.radiansToRotations(launchRad);
        }

        /** Convert mechanism rotations to launch angle (radians). */
        public static double mechanismRotationsToLaunchRad(double mechanismRotations) {
                return Units.rotationsToRadians(mechanismRotations);
        }

        static {
                // Flywheel Config
                kFlywheelConfig.Slot0.kP = .2;
                kFlywheelConfig.Slot0.kI = 0.0;
                kFlywheelConfig.Slot0.kD = 0.01;
                // Velocity loop is in mechanism units (SensorToMechanismRatio is set),
                // so kV must be volts per mechanism RPS (not motor RPS).
                // With 3:2 motor:wheel ratio and ~6000 RPM motor free speed:
                // mechanism free speed ~= 100 / 1.5 = 66.7 RPS -> kV ~= 12 / 66.7 = 0.18.
                kFlywheelConfig.Slot0.kV = 0.182;
                kFlywheelConfig.Slot0.kS = 0.0;
                kFlywheelConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
                kFlywheelConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
                kFlywheelConfig.Feedback.SensorToMechanismRatio = kFlywheelGearRatio;
                kFlywheelConfig.CurrentLimits
                                .withStatorCurrentLimit(kFlywheelStatorCurrentLimit)
                                .withStatorCurrentLimitEnable(true);

                // Flywheel Inverted Config (motors 2 & 4)
                kFlywheelInvertedConfig.Slot0.kP = .1;
                kFlywheelInvertedConfig.Slot0.kI = 0.0;
                kFlywheelInvertedConfig.Slot0.kD = 0.01;
                kFlywheelInvertedConfig.Slot0.kV = 0.182;
                kFlywheelInvertedConfig.Slot0.kS = 0.0;
                kFlywheelInvertedConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
                kFlywheelInvertedConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
                kFlywheelInvertedConfig.Feedback.SensorToMechanismRatio = kFlywheelGearRatio;
                kFlywheelInvertedConfig.CurrentLimits
                                .withStatorCurrentLimit(kFlywheelStatorCurrentLimit)
                                .withStatorCurrentLimitEnable(true);

                // Hood Config
                kHoodConfig.Slot0.kP = 40.0;
                kHoodConfig.Slot0.kI = 0.0;
                kHoodConfig.Slot0.kD = 0.0;
                kHoodConfig.Slot0.kV = 0.0;
                kHoodConfig.Slot0.kG = 0.2; // Gravity hold (Volts)
                kHoodConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
                kHoodConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
                kHoodConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
                kHoodConfig.Feedback.SensorToMechanismRatio = kHoodGearRatio;
                kHoodConfig.CurrentLimits
                                .withSupplyCurrentLimit(kHoodSupplyCurrentLimit)
                                .withSupplyCurrentLimitEnable(true);
        }
}
