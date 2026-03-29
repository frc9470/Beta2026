package com.team9470.subsystems.hopper;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.team254.lib.drivers.TalonFXFactory;
import com.team254.lib.drivers.TalonUtil;

import com.team9470.Ports;
import com.team9470.telemetry.TelemetryManager;
import com.team9470.telemetry.structs.HopperSnapshot;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static com.team9470.subsystems.hopper.HopperConstants.*;

/**
 * Hopper subsystem - high volume game piece feed to shooter.
 * Driven by 3 Kraken X44 motors (left, right, top). Only runs during shooting.
 */
public class Hopper extends SubsystemBase {

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

    // Control
    private final VoltageOut voltageRequest = new VoltageOut(0);
    private boolean running = false;

    // Status signals
    private final StatusSignal<AngularVelocity> leftVelocity;
    private final StatusSignal<Current> leftCurrent;
    private final TelemetryManager telemetry = TelemetryManager.getInstance();

    private Hopper() {
        hopperMotor = TalonFXFactory.createDefaultTalon(Ports.HOPPER_MOTOR);
        feederLeftMotor = TalonFXFactory.createDefaultTalon(Ports.FEEDER_LEFT);
        feederRightMotor = TalonFXFactory.createDefaultTalon(Ports.FEEDER_RIGHT);

        // Apply configurations
        TalonUtil.applyAndCheckConfiguration(hopperMotor, kHopperConfig);
        TalonUtil.applyAndCheckConfiguration(feederLeftMotor, kFeederLeftConfig);
        TalonUtil.applyAndCheckConfiguration(feederRightMotor, kFeederRightConfig);

        // Status signals
        leftVelocity = hopperMotor.getVelocity();
        leftCurrent = hopperMotor.getSupplyCurrent();

        BaseStatusSignal.setUpdateFrequencyForAll(50, leftVelocity, leftCurrent);
        hopperMotor.optimizeBusUtilization();
        feederLeftMotor.optimizeBusUtilization();
        feederRightMotor.optimizeBusUtilization();
    }

    @Override
    public void periodic() {
        double voltage = running ? kFeedVoltage : 0.0;
        hopperMotor.setControl(voltageRequest.withOutput(voltage));
        feederLeftMotor.setControl(voltageRequest.withOutput(voltage));
        feederRightMotor.setControl(voltageRequest.withOutput(voltage));

        telemetry.publishHopperState(new HopperSnapshot(
                running,
                voltage,
                leftVelocity.getValueAsDouble(),
                leftCurrent.getValueAsDouble()));
    }

    // --- Control ---

    public void setRunning(boolean run) {
        this.running = run;
    }

    public void run() {
        setRunning(true);
    }

    public void stop() {
        setRunning(false);
    }

    public boolean isRunning() {
        return running;
    }

    // --- Commands ---

    /**
     * Command to run the hopper while held.
     */
    public Command runCommand() {
        return this.startEnd(this::run, this::stop).withName("Hopper Run");
    }
}
