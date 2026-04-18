package com.team9470.subsystems.intake;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.team254.lib.drivers.TalonFXFactory;
import com.team254.lib.drivers.TalonUtil;

import com.team9470.Ports;
import com.team9470.Robot;
import com.team9470.simulation.IntakeSimulation;
import com.team9470.telemetry.TelemetryManager;
import com.team9470.telemetry.structs.IntakeSnapshot;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
    private final MotionMagicVoltage mmRequest = new MotionMagicVoltage(0).withSlot(0).withEnableFOC(true);
    private final VoltageOut pivotVoltageRequest = new VoltageOut(0).withEnableFOC(true);
    private final VoltageOut rollerVoltageRequest = new VoltageOut(0).withEnableFOC(false);

    // State
    private boolean deployed = false;
    private boolean deployHigh = false;
    private boolean agitating = false;
    private boolean awake = false; // Nothing moves until first deploy/agitate
    private boolean needsHoming = false; // Deferred to manual request (intake can't retract)
    private double homingStallStartTimestampSec = Double.NaN;
    private double agitateStartTimeSec = Double.NaN;
    private final TelemetryManager telemetry = TelemetryManager.getInstance();

    // Cached SmartDashboard gain values (pivot Slot0)
    private double cachedPivotKP, cachedPivotKI, cachedPivotKD;
    private double cachedPivotKV, cachedPivotKA, cachedPivotKG;
    // Cached SmartDashboard MotionMagic values
    private double cachedMMCruiseVel, cachedMMAccel, cachedMMJerk;

    private static final int STATE_SLEEPING = -1;
    private static final int STATE_HOMING = 0;
    private static final int STATE_RETRACTED = 1;
    private static final int STATE_DEPLOYED = 2;
    private static final int STATE_AGITATING = 3;
    private static final int STATE_DEPLOYED_HIGH = 5;

    private Intake() {
        // Apply configurations
        TalonUtil.applyAndCheckConfiguration(pivot, IntakeConstants.kPivotConfig);
        pivot.setPosition(0);

        TalonUtil.applyAndCheckConfiguration(leftRoller, IntakeConstants.kLeftRollerConfig);
        TalonUtil.applyAndCheckConfiguration(rightRoller, IntakeConstants.kRightRollerConfig);

        // rightRoller follows leftRoller with opposed direction (they have opposite inversions)
        rightRoller.setControl(new Follower(leftRoller.getDeviceID(), MotorAlignmentValue.Opposed));

        initSmartDashboardGains();
    }

    // --- Commands ---

    public void setDeployed(boolean deployed) {
        this.deployed = deployed;
        if (deployed) {
            this.deployHigh = false;
            this.awake = true;
        }
    }

    public void setDeployHigh(boolean deployHigh) {
        this.deployHigh = deployHigh;
        if (deployHigh) {
            this.deployed = false;
            this.awake = true;
        }
    }

    public void setAgitating(boolean agitating) {
        this.agitating = agitating;
        if (agitating) {
            agitateStartTimeSec = Timer.getFPGATimestamp();
            this.awake = true;
        } else {
            agitateStartTimeSec = Double.NaN;
        }
    }

    /**
     * Request intake pivot re-homing against its retract hardstop.
     */
    public void requestHome() {
        deployed = false;
        deployHigh = false;
        agitating = false;
        homingStallStartTimestampSec = Double.NaN;
        agitateStartTimeSec = Double.NaN;
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

    /** Agitate by slowly compressing upward while held. */
    public Command getAgitateCommand() {
        return this.startEnd(
                () -> setAgitating(true),
                () -> { setAgitating(false); setDeployed(true); })
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
                () -> leftRoller.setControl(rollerVoltageRequest.withOutput(-IntakeConstants.kRollerVoltage)),
                () -> leftRoller.setControl(rollerVoltageRequest.withOutput(0)))
                .withName("Intake Outtake");
    }

    // ==================== SmartDashboard PID Tuning ====================

    private void initSmartDashboardGains() {
        // Pivot Slot0 gains from config
        cachedPivotKP = IntakeConstants.kPivotConfig.Slot0.kP;
        cachedPivotKI = IntakeConstants.kPivotConfig.Slot0.kI;
        cachedPivotKD = IntakeConstants.kPivotConfig.Slot0.kD;
        cachedPivotKV = IntakeConstants.kPivotConfig.Slot0.kV;
        cachedPivotKA = IntakeConstants.kPivotConfig.Slot0.kA;
        cachedPivotKG = IntakeConstants.kPivotConfig.Slot0.kG;
        SmartDashboard.putNumber("IntakePivot/kP", cachedPivotKP);
        SmartDashboard.putNumber("IntakePivot/kI", cachedPivotKI);
        SmartDashboard.putNumber("IntakePivot/kD", cachedPivotKD);
        SmartDashboard.putNumber("IntakePivot/kV", cachedPivotKV);
        SmartDashboard.putNumber("IntakePivot/kA", cachedPivotKA);
        SmartDashboard.putNumber("IntakePivot/kG", cachedPivotKG);

        // MotionMagic params
        cachedMMCruiseVel = IntakeConstants.kPivotConfig.MotionMagic.MotionMagicCruiseVelocity;
        cachedMMAccel = IntakeConstants.kPivotConfig.MotionMagic.MotionMagicAcceleration;
        cachedMMJerk = IntakeConstants.kPivotConfig.MotionMagic.MotionMagicJerk;
        SmartDashboard.putNumber("IntakePivot/MM_CruiseVel", cachedMMCruiseVel);
        SmartDashboard.putNumber("IntakePivot/MM_Accel", cachedMMAccel);
        SmartDashboard.putNumber("IntakePivot/MM_Jerk", cachedMMJerk);
    }

    private void updateSmartDashboardGains() {
        // --- Pivot Slot0 gains ---
        double pivotKP = SmartDashboard.getNumber("IntakePivot/kP", cachedPivotKP);
        double pivotKI = SmartDashboard.getNumber("IntakePivot/kI", cachedPivotKI);
        double pivotKD = SmartDashboard.getNumber("IntakePivot/kD", cachedPivotKD);
        double pivotKV = SmartDashboard.getNumber("IntakePivot/kV", cachedPivotKV);
        double pivotKA = SmartDashboard.getNumber("IntakePivot/kA", cachedPivotKA);
        double pivotKG = SmartDashboard.getNumber("IntakePivot/kG", cachedPivotKG);
        if (pivotKP != cachedPivotKP || pivotKI != cachedPivotKI || pivotKD != cachedPivotKD
                || pivotKV != cachedPivotKV || pivotKA != cachedPivotKA || pivotKG != cachedPivotKG) {
            cachedPivotKP = pivotKP;
            cachedPivotKI = pivotKI;
            cachedPivotKD = pivotKD;
            cachedPivotKV = pivotKV;
            cachedPivotKA = pivotKA;
            cachedPivotKG = pivotKG;
            Slot0Configs newPivotGains = new Slot0Configs()
                    .withKP(pivotKP).withKI(pivotKI).withKD(pivotKD)
                    .withKV(pivotKV).withKA(pivotKA).withKG(pivotKG);
            pivot.getConfigurator().apply(newPivotGains);
        }

        // --- MotionMagic params ---
        double mmCruiseVel = SmartDashboard.getNumber("IntakePivot/MM_CruiseVel", cachedMMCruiseVel);
        double mmAccel = SmartDashboard.getNumber("IntakePivot/MM_Accel", cachedMMAccel);
        double mmJerk = SmartDashboard.getNumber("IntakePivot/MM_Jerk", cachedMMJerk);
        if (mmCruiseVel != cachedMMCruiseVel || mmAccel != cachedMMAccel || mmJerk != cachedMMJerk) {
            cachedMMCruiseVel = mmCruiseVel;
            cachedMMAccel = mmAccel;
            cachedMMJerk = mmJerk;
            MotionMagicConfigs newMM = new MotionMagicConfigs()
                    .withMotionMagicCruiseVelocity(mmCruiseVel)
                    .withMotionMagicAcceleration(mmAccel)
                    .withMotionMagicJerk(mmJerk);
            pivot.getConfigurator().apply(newMM);
        }
    }

    @Override
    public void periodic() {
        // Sleep until first deploy — no motors run on boot.
        if (!awake) {
            pivot.setControl(pivotVoltageRequest.withOutput(0));
            leftRoller.setControl(rollerVoltageRequest.withOutput(0));
            telemetry.publishIntakeState(new IntakeSnapshot(
                    false,
                    false,
                    false,
                    STATE_SLEEPING,
                    0.0,
                    0.0,
                    IntakeConstants.pivotMechanismRotationsToAngle(pivot.getPosition().getValueAsDouble()).in(Radians),
                    0.0,
                    0.0,
                    pivot.getSupplyCurrent().getValueAsDouble(),
                    pivot.getStatorCurrent().getValueAsDouble(),
                    0.0,
                    0.0,
                    leftRoller.getSupplyCurrent().getValueAsDouble(),
                    leftRoller.getStatorCurrent().getValueAsDouble(),
                    rightRoller.getStatorCurrent().getValueAsDouble()));
            return;
        }

        if (needsHoming) {
            // Drive toward retract hardstop
            pivot.setControl(pivotVoltageRequest.withOutput(IntakeConstants.kHomingVoltage));
            leftRoller.setControl(rollerVoltageRequest.withOutput(0));

            // Check for a sustained stall at the retract hardstop.
            double now = Timer.getFPGATimestamp();
            double supplyCurrent = pivot.getSupplyCurrent().getValueAsDouble();
            double statorCurrent = pivot.getStatorCurrent().getValueAsDouble();
            double velocity = Math.abs(pivot.getVelocity().getValueAsDouble());
            boolean stalled = statorCurrent > IntakeConstants.kStallCurrentThreshold
                    && velocity < IntakeConstants.kStallVelocityThreshold;
            if (stalled) {
                if (Double.isNaN(homingStallStartTimestampSec)) {
                    homingStallStartTimestampSec = now;
                }
                if (now - homingStallStartTimestampSec >= IntakeConstants.kStallTimeThreshold) {
                    pivot.setControl(pivotVoltageRequest.withOutput(0));
                    pivot.setPosition(IntakeConstants.pivotAngleToMechanismRotations(IntakeConstants.kHomePosition));
                    homingStallStartTimestampSec = Double.NaN;
                    needsHoming = false;
                }
            } else {
                homingStallStartTimestampSec = Double.NaN;
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
                    supplyCurrent,
                    statorCurrent,
                    IntakeConstants.kHomingVoltage,
                    0.0,
                    // TODO: Make it check from both rollers.
                    leftRoller.getSupplyCurrent().getValueAsDouble(),
                    leftRoller.getStatorCurrent().getValueAsDouble(),
                    rightRoller.getStatorCurrent().getValueAsDouble()));
            return;
        }

        // --- Normal operation ---

        // Pivot control
        Angle targetAngle;
        if (deployHigh) {
            targetAngle = IntakeConstants.kDeployHighAngle;
        } else if (agitating) {
            double nowSec = Timer.getFPGATimestamp();
            if (Double.isNaN(agitateStartTimeSec)) {
                agitateStartTimeSec = nowSec;
            }
            double elapsed = nowSec - agitateStartTimeSec;
            double period = 1.0 / IntakeConstants.kAgitateFrequencyHz;
            double phase = (elapsed % period) / period;
            if (phase < 1.0 / 3.0) {
                targetAngle = IntakeConstants.kAgitateMiddleAngle.div(2);
            } else if (phase < 2.0 / 3.0) {
                targetAngle = IntakeConstants.kDeployAngle;
            } else {
                targetAngle = IntakeConstants.kAgitateMiddleAngle;
            }
        } else if (deployed) {
            targetAngle = IntakeConstants.kDeployAngle;
        } else {
            targetAngle = IntakeConstants.kRetractAngle;
        }
        double targetRot = IntakeConstants.pivotAngleToMechanismRotations(targetAngle);
        pivot.setControl(mmRequest.withPosition(targetRot));

        // Roller control
        double rollerVolts = (deployed || deployHigh || agitating) ? IntakeConstants.kRollerVoltage : 0.0;
        leftRoller.setControl(rollerVoltageRequest.withOutput(rollerVolts));

        // --- Telemetry ---
        double currentPositionRot = pivot.getPosition().getValueAsDouble();
        double currentPositionRad = IntakeConstants.pivotMechanismRotationsToAngle(currentPositionRot)
                .in(Radians);
        double goalRad = targetAngle.in(Radians);
        double setpointRad = IntakeConstants.pivotMechanismRotationsToAngle(targetRot).in(Radians);

        int stateCode;
        if (agitating) {
            stateCode = STATE_AGITATING;
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
                pivot.getStatorCurrent().getValueAsDouble(),
                pivot.getMotorVoltage().getValueAsDouble(),
                rollerVolts,
                // TODO: Make it publish both left and right roller.
                leftRoller.getSupplyCurrent().getValueAsDouble(),
                leftRoller.getStatorCurrent().getValueAsDouble(),
                rightRoller.getStatorCurrent().getValueAsDouble()));

        // Update visualization (works in both real and sim)
        if (Robot.isSimulation()) {
            IntakeSimulation.getInstance().updateVisualization(currentPositionRot);
        }

        updateSmartDashboardGains();
    }

    @Override
    public void simulationPeriodic() {
        // TODO: Make it simulate both left and right roller.
        IntakeSimulation.getInstance().update(pivot, leftRoller, deployed || deployHigh || agitating);
    }

    // --- Accessors for physics ---

    public boolean isRunning() {
        return awake && (deployed || deployHigh || agitating);
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
