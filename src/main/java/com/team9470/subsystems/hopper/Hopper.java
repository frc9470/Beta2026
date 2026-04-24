package com.team9470.subsystems.hopper;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.team254.lib.drivers.TalonFXFactory;
import com.team254.lib.drivers.TalonUtil;

import com.team9470.Ports;
import com.team9470.telemetry.TelemetryManager;
import com.team9470.telemetry.structs.HopperSnapshot;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static com.team9470.subsystems.hopper.HopperConstants.*;

/**
 * Hopper subsystem - high volume game piece feed to shooter.
 * Driven by 3 Kraken X44 motors (left, right, top). Only runs during shooting.
 */
public class Hopper extends SubsystemBase {
    private static final double kTopBeamBreakDebounceSec = 0.025;
    private static final double kVoltageEpsilon = 1e-4;

    private static Hopper instance;

    public static Hopper getInstance() {
        if (instance == null) {
            instance = new Hopper();
        }
        return instance;
    }

    // Hardware
    private final TalonFX hopperMotor;
    private final TalonFX feederLeftMotor;
    private final TalonFX feederRightMotor;
    private final DigitalInput topBeamBreak;

    // Control
    private final VoltageOut hopperVoltageRequest = new VoltageOut(0).withEnableFOC(true);
    private final VoltageOut feederVoltageRequest = new VoltageOut(0).withEnableFOC(true);
    private double commandedHopperVolts = 0.0;
    private double commandedFeederVolts = 0.0;
    private boolean topBeamBreakRawBlocked;
    private boolean topBeamBreakBlocked;
    private double topBeamBreakLastRawChangeSec;

    // Status signals
    private final StatusSignal<AngularVelocity> hopperVelocity;
    private final StatusSignal<Current> hopperCurrent;
    private final StatusSignal<Current> hopperStatorCurrent;
    private final StatusSignal<AngularVelocity> feederLeftVelocity;
    private final StatusSignal<Current> feederLeftSupplyCurrent;
    private final StatusSignal<Current> feederLeftStatorCurrent;
    private final StatusSignal<Voltage> feederLeftAppliedVolts;
    private final StatusSignal<AngularVelocity> feederRightVelocity;
    private final StatusSignal<Current> feederRightSupplyCurrent;
    private final StatusSignal<Current> feederRightStatorCurrent;
    private final StatusSignal<Voltage> feederRightAppliedVolts;
    private final TelemetryManager telemetry = TelemetryManager.getInstance();

    private Hopper() {
        hopperMotor = TalonFXFactory.createDefaultTalon(Ports.HOPPER_MOTOR);
        feederLeftMotor = TalonFXFactory.createDefaultTalon(Ports.FEEDER_LEFT);
        feederRightMotor = TalonFXFactory.createDefaultTalon(Ports.FEEDER_RIGHT);
        topBeamBreak = new DigitalInput(Ports.FEEDER_TOP_BEAM_BREAK_DIO);

        // Apply configurations
        TalonUtil.applyAndCheckConfiguration(hopperMotor, kHopperConfig);
        TalonUtil.applyAndCheckConfiguration(feederLeftMotor, kFeederLeftConfig);
        TalonUtil.applyAndCheckConfiguration(feederRightMotor, kFeederRightConfig);

        // Right feeder mirrors the left feeder across an opposed physical layout.
        feederRightMotor.setControl(new Follower(feederLeftMotor.getDeviceID(), MotorAlignmentValue.Opposed));

        // Status signals
        hopperVelocity = hopperMotor.getVelocity();
        hopperCurrent = hopperMotor.getSupplyCurrent();
        hopperStatorCurrent = hopperMotor.getStatorCurrent();
        feederLeftVelocity = feederLeftMotor.getVelocity();
        feederLeftSupplyCurrent = feederLeftMotor.getSupplyCurrent();
        feederLeftStatorCurrent = feederLeftMotor.getStatorCurrent();
        feederLeftAppliedVolts = feederLeftMotor.getMotorVoltage();
        feederRightVelocity = feederRightMotor.getVelocity();
        feederRightSupplyCurrent = feederRightMotor.getSupplyCurrent();
        feederRightStatorCurrent = feederRightMotor.getStatorCurrent();
        feederRightAppliedVolts = feederRightMotor.getMotorVoltage();

        BaseStatusSignal.setUpdateFrequencyForAll(
                50,
                hopperVelocity,
                hopperCurrent,
                hopperStatorCurrent,
                feederLeftVelocity,
                feederLeftSupplyCurrent,
                feederLeftStatorCurrent,
                feederLeftAppliedVolts,
                feederRightVelocity,
                feederRightSupplyCurrent,
                feederRightStatorCurrent,
                feederRightAppliedVolts);
        hopperMotor.optimizeBusUtilization();
        feederLeftMotor.optimizeBusUtilization();
        feederRightMotor.optimizeBusUtilization();

        topBeamBreakRawBlocked = readTopBeamBreakRawBlocked();
        topBeamBreakBlocked = topBeamBreakRawBlocked;
        topBeamBreakLastRawChangeSec = Timer.getFPGATimestamp();
    }

    @Override
    public void periodic() {
        if (!Utils.isSimulation()) {
            BaseStatusSignal.refreshAll(
                    hopperVelocity,
                    hopperCurrent,
                    hopperStatorCurrent,
                    feederLeftVelocity,
                    feederLeftSupplyCurrent,
                    feederLeftStatorCurrent,
                    feederLeftAppliedVolts,
                    feederRightVelocity,
                    feederRightSupplyCurrent,
                    feederRightStatorCurrent,
                    feederRightAppliedVolts);
        }
        updateTopBeamBreakState();

        hopperMotor.setControl(hopperVoltageRequest.withOutput(commandedHopperVolts));
        feederLeftMotor.setControl(feederVoltageRequest.withOutput(commandedFeederVolts));

        telemetry.publishHopperState(new HopperSnapshot(
                isRunning(),
                commandedHopperVolts,
                commandedFeederVolts,
                hopperVelocity.getValueAsDouble(),
                hopperCurrent.getValueAsDouble(),
                topBeamBreakRawBlocked,
                topBeamBreakBlocked));
        telemetry.publishHopperFeederState(
                commandedFeederVolts,
                feederLeftVelocity.getValueAsDouble(),
                feederLeftSupplyCurrent.getValueAsDouble(),
                feederLeftStatorCurrent.getValueAsDouble(),
                feederLeftAppliedVolts.getValueAsDouble(),
                feederRightVelocity.getValueAsDouble(),
                feederRightSupplyCurrent.getValueAsDouble(),
                feederRightStatorCurrent.getValueAsDouble(),
                feederRightAppliedVolts.getValueAsDouble());
    }

    // --- Control ---

    private boolean readTopBeamBreakRawBlocked() {
        // Beam breaks are wired active-low: false when the beam is interrupted.
        return !topBeamBreak.get();
    }

    private void updateTopBeamBreakState() {
        boolean rawBlocked = readTopBeamBreakRawBlocked();
        double nowSec = Timer.getFPGATimestamp();
        if (rawBlocked != topBeamBreakRawBlocked) {
            topBeamBreakRawBlocked = rawBlocked;
            topBeamBreakLastRawChangeSec = nowSec;
        }
        if (topBeamBreakBlocked != topBeamBreakRawBlocked
                && nowSec - topBeamBreakLastRawChangeSec >= kTopBeamBreakDebounceSec) {
            topBeamBreakBlocked = topBeamBreakRawBlocked;
        }
    }

    public void setHopperVoltage(double volts) {
        commandedHopperVolts = volts;
    }

    public void setFeederVoltage(double volts) {
        commandedFeederVolts = volts;
    }

    public void stopAll() {
        commandedHopperVolts = 0.0;
        commandedFeederVolts = 0.0;
    }

    public void setRunning(boolean run) {
        if (run) {
            setHopperVoltage(kFeedVoltage);
            setFeederVoltage(kFeedVoltage);
        } else {
            stopAll();
        }
    }

    public void run() {
        setRunning(true);
    }

    public void stop() {
        stopAll();
    }

    public boolean isRunning() {
        return Math.abs(commandedHopperVolts) > kVoltageEpsilon || Math.abs(commandedFeederVolts) > kVoltageEpsilon;
    }

    public boolean isTopBeamBreakRawBlocked() {
        return topBeamBreakRawBlocked;
    }

    public boolean isTopBeamBreakBlocked() {
        return topBeamBreakBlocked;
    }

    public double getFeederLeftVelocityRps() {
        return feederLeftVelocity.getValueAsDouble();
    }

    public double getHopperVelocityRps() {
        return hopperVelocity.getValueAsDouble();
    }

    public double getHopperStatorCurrentAmps() {
        return hopperStatorCurrent.getValueAsDouble();
    }

    public double getFeederRightVelocityRps() {
        return feederRightVelocity.getValueAsDouble();
    }

    public double getFeederLeftStatorCurrentAmps() {
        return feederLeftStatorCurrent.getValueAsDouble();
    }

    public double getFeederRightStatorCurrentAmps() {
        return feederRightStatorCurrent.getValueAsDouble();
    }

    // --- Commands ---

    /**
     * Command to run the hopper while held.
     */
    public Command runCommand() {
        return this.startEnd(this::run, this::stop).withName("Hopper Run");
    }
}
