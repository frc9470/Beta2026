package com.team9470.subsystems.intake;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.team254.lib.drivers.TalonFXFactory;
import com.team254.lib.drivers.TalonUtil;

import com.team9470.Ports;
import com.team9470.Robot;
import com.team9470.simulation.IntakeSimulation;
import com.team9470.telemetry.TelemetryManager;
import com.team9470.telemetry.structs.IntakeSnapshot;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static edu.wpi.first.units.Units.*;

/**
 * Intake subsystem - controls pivot arm and roller.
 * This class contains only the logic that runs on both real robot and
 * simulation.
 * All physics simulation is handled by IntakeSimulation.
 */
public class Intake extends SubsystemBase {

    private static Intake instance;

    public static Intake getInstance() {
        if (instance == null) {
            instance = new Intake();
        }
        return instance;
    }

    // Hardware
    private final TalonFX pivot = TalonFXFactory.createDefaultTalon(Ports.INTAKE_PIVOT);
    private final TalonFX leftRoller = TalonFXFactory.createDefaultTalon(Ports.INTAKE_ROLLER_LEFT);
    private final TalonFX rightRoller = TalonFXFactory.createDefaultTalon(Ports.INTAKE_ROLLER_RIGHT);

    // Controls
    private final MotionMagicVoltage mmRequest = new MotionMagicVoltage(0).withSlot(0);
    private final VoltageOut voltRequest = new VoltageOut(0);

    // State
    private boolean deployed = false;
    private boolean deployHigh = false;
    private boolean agitating = false;
    private boolean shooting = false;
    private double shootingStartTimestampSec = Double.NEGATIVE_INFINITY;
    private boolean needsHoming = true;
    private final TelemetryManager telemetry = TelemetryManager.getInstance();

    private static final int STATE_HOMING = 0;
    private static final int STATE_RETRACTED = 1;
    private static final int STATE_DEPLOYED = 2;
    private static final int STATE_AGITATE_DEPLOY = 3;
    private static final int STATE_AGITATE_MID = 4;
    private static final int STATE_DEPLOYED_HIGH = 5;
    private static final int STATE_SHOOTING_OVERRIDE = 6;

    private Intake() {
        // Apply configurations
        TalonUtil.applyAndCheckConfiguration(pivot, IntakeConstants.kPivotConfig);
        pivot.setPosition(0);

        TalonUtil.applyAndCheckConfiguration(leftRoller, IntakeConstants.kLeftRollerConfig);
        TalonUtil.applyAndCheckConfiguration(leftRoller, IntakeConstants.kRightRollerConfig);
    }

    // --- Commands ---

    public void setDeployed(boolean deployed) {
        this.deployed = deployed;
        if (deployed) {
            this.deployHigh = false;
        }
    }

    public void setDeployHigh(boolean deployHigh) {
        this.deployHigh = deployHigh;
        if (deployHigh) {
            this.deployed = false;
        }
    }

    public void setAgitating(boolean agitating) {
        this.agitating = agitating;
    }

    public void setShooting(boolean shooting) {
        if (shooting && !this.shooting) {
            shootingStartTimestampSec = Timer.getFPGATimestamp();
        } else if (!shooting) {
            shootingStartTimestampSec = Double.NEGATIVE_INFINITY;
        }
        this.shooting = shooting;
    }

    private boolean isAgitationActive() {
        if (!agitating) {
            return false;
        }
        if (!shooting) {
            return true;
        }
        return Timer.getFPGATimestamp() - shootingStartTimestampSec >= IntakeConstants.kShootAgitationDelaySec;
    }

    /**
     * Request intake pivot re-homing against its retract hardstop.
     */
    public void requestHome() {
        deployed = false;
        deployHigh = false;
        agitating = false;
        shooting = false;
        shootingStartTimestampSec = Double.NEGATIVE_INFINITY;
        needsHoming = true;
    }

    /** Toggle deploy/retract arm position. */
    public Command getToggleCommand() {
        return this.runOnce(() -> setDeployed(!deployed))
                .withName("Intake Toggle");
    }

    /** Toggle deploy-high/retract arm position. */
    public Command getDeployHighToggleCommand() {
        return this.runOnce(() -> setDeployHigh(!deployHigh))
                .withName("Intake Toggle High");
    }

    /** Agitate between deploy and middle position while held. */
    public Command getAgitateCommand() {
        return this.startEnd(
                () -> setAgitating(true),
                () -> setAgitating(false))
                .withName("Intake Agitate");
    }

    /** Deploy + run rollers while held, retract on release. */
    public Command getIntakeCommand() {
        return this.startEnd(
                () -> setDeployed(true),
                () -> setDeployed(false))
                .withName("Intake Run");
    }

    /** Reverse rollers while held (for clearing jams). */
    public Command getOuttakeCommand() {
        return this.startEnd(
                () -> {
                    leftRoller.setControl(voltRequest.withOutput(-IntakeConstants.kRollerVoltage));
                    rightRoller.setControl(voltRequest.withOutput(-IntakeConstants.kRollerVoltage));
                },
                () -> {
                    leftRoller.setControl(voltRequest.withOutput(0));
                    rightRoller.setControl(voltRequest.withOutput(0));
                })
                .withName("Intake Outtake");
    }

    @Override
    public void periodic() {
        if (needsHoming) {
            // Drive toward retract hardstop
            pivot.setControl(voltRequest.withOutput(IntakeConstants.kHomingVoltage));
            leftRoller.setControl(voltRequest.withOutput(0));
            rightRoller.setControl(voltRequest.withOutput(0));

            // Check for stall (hit hardstop): high current AND low velocity
            double current = pivot.getSupplyCurrent().getValueAsDouble();
            double velocity = Math.abs(pivot.getVelocity().getValueAsDouble());
            if (current > IntakeConstants.kStallCurrentThreshold && velocity < 0.5) {
                pivot.setControl(voltRequest.withOutput(0));
                pivot.setPosition(IntakeConstants.pivotAngleToMechanismRotations(IntakeConstants.kHomePosition));
                needsHoming = false;
            }

            telemetry.publishIntakeState(new IntakeSnapshot(
                    true,
                    deployed || deployHigh,
                    agitating,
                    STATE_HOMING,
                    0.0,
                    0.0,
                    IntakeConstants.pivotMechanismRotationsToAngle(pivot.getPosition().getValueAsDouble()).in(Radians),
                    0.0,
                    velocity * 2.0 * Math.PI,
                    current,
                    IntakeConstants.kHomingVoltage,
                    0.0,
                    // TODO: Make it check from both rollers.
                    leftRoller.getSupplyCurrent().getValueAsDouble()));
            return;
        }

        // --- Normal operation ---

        // Pivot control
        boolean effectiveAgitating = isAgitationActive();
        boolean shootingDelayActive = agitating && shooting && !effectiveAgitating;
        boolean agitateAtDeploy = false;
        if (effectiveAgitating) {
            double t = Timer.getFPGATimestamp();
            agitateAtDeploy = Math.sin(2.0 * Math.PI * IntakeConstants.kAgitateFrequencyHz * t) >= 0.0;
        }
        Angle targetAngle;
        if (effectiveAgitating) {
            targetAngle = agitateAtDeploy ? IntakeConstants.kDeployAngle : IntakeConstants.kAgitateMiddleAngle;
        } else if (deployHigh) {
            targetAngle = IntakeConstants.kDeployHighAngle;
        } else if (deployed) {
            targetAngle = IntakeConstants.kDeployAngle;
        } else {
            targetAngle = IntakeConstants.kRetractAngle;
        }
        double targetRot = IntakeConstants.pivotAngleToMechanismRotations(targetAngle);
        pivot.setControl(mmRequest.withPosition(targetRot));

        // Roller control
        double rollerVolts = (deployed || deployHigh || effectiveAgitating) ? IntakeConstants.kRollerVoltage : 0.0;
        leftRoller.setControl(voltRequest.withOutput(rollerVolts));
        rightRoller.setControl(voltRequest.withOutput(rollerVolts));

        // --- Telemetry ---
        double currentPositionRot = pivot.getPosition().getValueAsDouble();
        double currentPositionRad = IntakeConstants.pivotMechanismRotationsToAngle(currentPositionRot)
                .in(Radians);
        double goalRad = targetAngle.in(Radians);
        double setpointRad = IntakeConstants.pivotMechanismRotationsToAngle(targetRot).in(Radians);

        int stateCode;
        if (shootingDelayActive) {
            stateCode = STATE_SHOOTING_OVERRIDE;
        } else if (effectiveAgitating) {
            stateCode = agitateAtDeploy ? STATE_AGITATE_DEPLOY : STATE_AGITATE_MID;
        } else if (deployHigh) {
            stateCode = STATE_DEPLOYED_HIGH;
        } else if (deployed) {
            stateCode = STATE_DEPLOYED;
        } else {
            stateCode = STATE_RETRACTED;
        }
        telemetry.publishIntakeState(new IntakeSnapshot(
                false,
                deployed || deployHigh,
                agitating,
                stateCode,
                goalRad,
                setpointRad,
                currentPositionRad,
                setpointRad - currentPositionRad,
                pivot.getVelocity().getValueAsDouble() * 2.0 * Math.PI,
                pivot.getSupplyCurrent().getValueAsDouble(),
                pivot.getMotorVoltage().getValueAsDouble(),
                rollerVolts,
                // TODO: Make it publish both left and right roller.
                leftRoller.getSupplyCurrent().getValueAsDouble()));

        // Update visualization (works in both real and sim)
        if (Robot.isSimulation()) {
            IntakeSimulation.getInstance().updateVisualization(currentPositionRot);
        }
    }

    @Override
    public void simulationPeriodic() {
        // TODO: Make it simulate both left and right roller.
        IntakeSimulation.getInstance().update(pivot, leftRoller, deployed || deployHigh || isAgitationActive());
    }

    // --- Accessors for physics ---

    public boolean isRunning() {
        return deployed || deployHigh || isAgitationActive();
    }

    public boolean isDeployed() {
        return deployed || deployHigh;
    }

    public boolean isDeployHigh() {
        return deployHigh;
    }

    public boolean isAgitating() {
        return agitating;
    }

    public double getPivotAngle() {
        if (Robot.isSimulation()) {
            return IntakeSimulation.getInstance().getAngleRad();
        }
        return IntakeConstants.pivotMechanismRotationsToAngle(pivot.getPosition().getValueAsDouble())
                .in(Radians);
    }

    /**
     * Returns whether homing is complete.
     */
    public boolean isHomed() {
        return !needsHoming;
    }
}
