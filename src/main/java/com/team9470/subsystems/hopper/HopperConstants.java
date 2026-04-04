package com.team9470.subsystems.hopper;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class HopperConstants {
        // NOTE: CAN IDs are in Ports.java (HOPPER_LEFT, HOPPER_RIGHT, HOPPER_TOP)

        // Control
        public static final double kFeedVoltage = -12.0; // Voltage when feeding to shooter
        public static final double kRollerSupplyCurrentLimit = 25.0;
        public static final double kTopStatorCurrentLimit = 50.0;

        // Motor Configs
        public static final TalonFXConfiguration kHopperConfig = new TalonFXConfiguration();
        public static final TalonFXConfiguration kFeederLeftConfig = new TalonFXConfiguration();
        public static final TalonFXConfiguration kFeederRightConfig = new TalonFXConfiguration();

        static {
                // Hopper Motor Config
                kHopperConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
                // TODO: Verify
                kHopperConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
                kHopperConfig.CurrentLimits
                                .withSupplyCurrentLimit(kRollerSupplyCurrentLimit)
                                .withSupplyCurrentLimitEnable(true);

                kFeederLeftConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
                kFeederLeftConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
                kFeederLeftConfig.CurrentLimits
                                .withSupplyCurrentLimitEnable(false)
                                .withStatorCurrentLimit(kTopStatorCurrentLimit)
                                .withStatorCurrentLimitEnable(true);

                // Right feeder motor config (opposite orientation to left motor)
                kFeederRightConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
                kFeederRightConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
                kFeederRightConfig.CurrentLimits
                                .withSupplyCurrentLimitEnable(false)
                                .withStatorCurrentLimit(kTopStatorCurrentLimit)
                                .withStatorCurrentLimitEnable(true);
        }
}
