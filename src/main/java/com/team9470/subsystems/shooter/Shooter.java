package com.team9470.subsystems.shooter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.team254.lib.drivers.TalonFXFactory;
import com.team254.lib.drivers.TalonUtil;

import com.team9470.Ports;
import com.team9470.Robot;
import com.team9470.simulation.ProjectileSimulation;
import com.team9470.subsystems.shooter.characterization.ShooterCharacterizationConfig;
import com.team9470.subsystems.shooter.characterization.ShooterCharacterizationCsvLogger;
import com.team9470.subsystems.shooter.characterization.ShooterCharacterizationMode;
import com.team9470.subsystems.shooter.characterization.ShooterCharacterizationSession;
import com.team9470.subsystems.shooter.characterization.ShooterCharacterizationStatus;
import com.team9470.telemetry.TelemetryManager;
import com.team9470.telemetry.structs.ShooterCharacterizationSnapshot;
import com.team9470.telemetry.structs.ShooterSnapshot;
import com.team9470.util.AutoAim.ShootingSolution;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.io.IOException;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.function.Supplier;

/**
 * Shooter subsystem - controls flywheel velocity and hood angle.
 * This class contains only the logic that runs on both real robot and
 * simulation.
 * All physics simulation is handled by ProjectileSimulation.
 */
public class Shooter extends SubsystemBase {
    private static final double kFlywheelSetpointToleranceRPS = 0.7; // 42 RPM
    private static final double kHoodSetpointToleranceRotations = 0.01; // ~3.6 degrees
    private static final double kRestingFlywheelRPS = 2000.0 / 60.0;
    private static final double kOverrevOffsetRPS = 125.0 / 60.0; // +125 RPM
    private static final double kOverrevRampSeconds = 0.75;
    private static final double kNonZeroSpeedEpsilonRPS = 1e-4;
    private static final double kCharacterizationMinBatteryVolts = 9.5;
    private static final double kCharacterizationMaxCurrentAmps = 240.0;
    private static final double kCharacterizationCurrentDebounceSec = 0.15;
    private static final double kCharacterizationMaxFlywheelRpm = ShooterCharacterizationConfig.defaultMaxFlywheelRpm() + 250.0;
    private static final AtomicInteger kCharacterizationRunIds = new AtomicInteger(1);

    private static final double kMinHoodAngleRotations = ShooterConstants.launchRadToMechanismRotations(
            ShooterConstants.kMinHoodAngle.in(edu.wpi.first.units.Units.Radians));

    // Hardware
    private final TalonFX flywheel1 = TalonFXFactory.createDefaultTalon(Ports.FLYWHEEL_1);
    private final TalonFX flywheel2 = TalonFXFactory.createDefaultTalon(Ports.FLYWHEEL_2);
    private final TalonFX flywheel3 = TalonFXFactory.createDefaultTalon(Ports.FLYWHEEL_3);
    private final TalonFX flywheel4 = TalonFXFactory.createDefaultTalon(Ports.FLYWHEEL_4);
    private final TalonFX hoodMotor = TalonFXFactory.createDefaultTalon(Ports.HOOD_MOTOR);
    private final StatusSignal<AngularVelocity> flywheelVelocitySignal = flywheel1.getVelocity();
    private final StatusSignal<Current> flywheel1SupplyCurrentSignal = flywheel1.getSupplyCurrent();
    private final StatusSignal<Current> flywheel2SupplyCurrentSignal = flywheel2.getSupplyCurrent();
    private final StatusSignal<Current> flywheel3SupplyCurrentSignal = flywheel3.getSupplyCurrent();
    private final StatusSignal<Current> flywheel4SupplyCurrentSignal = flywheel4.getSupplyCurrent();
    private final StatusSignal<Current> flywheel1StatorCurrentSignal = flywheel1.getStatorCurrent();
    private final StatusSignal<Current> flywheel2StatorCurrentSignal = flywheel2.getStatorCurrent();
    private final StatusSignal<Current> flywheel3StatorCurrentSignal = flywheel3.getStatorCurrent();
    private final StatusSignal<Current> flywheel4StatorCurrentSignal = flywheel4.getStatorCurrent();
    private final StatusSignal<Voltage> flywheel1MotorVoltageSignal = flywheel1.getMotorVoltage();
    private final StatusSignal<Voltage> flywheel2MotorVoltageSignal = flywheel2.getMotorVoltage();
    private final StatusSignal<Voltage> flywheel3MotorVoltageSignal = flywheel3.getMotorVoltage();
    private final StatusSignal<Voltage> flywheel4MotorVoltageSignal = flywheel4.getMotorVoltage();
    private final BaseStatusSignal[] characterizationSignals = new BaseStatusSignal[] {
            flywheelVelocitySignal,
            flywheel1SupplyCurrentSignal,
            flywheel2SupplyCurrentSignal,
            flywheel3SupplyCurrentSignal,
            flywheel4SupplyCurrentSignal,
            flywheel1StatorCurrentSignal,
            flywheel2StatorCurrentSignal,
            flywheel3StatorCurrentSignal,
            flywheel4StatorCurrentSignal,
            flywheel1MotorVoltageSignal,
            flywheel2MotorVoltageSignal,
            flywheel3MotorVoltageSignal,
            flywheel4MotorVoltageSignal
    };

    // Controls
    private final VelocityVoltage flywheelRequest = new VelocityVoltage(0).withEnableFOC(false);
    private final PositionVoltage hoodRequest = new PositionVoltage(0);
    private final VoltageOut flywheelVoltageRequest = new VoltageOut(0).withEnableFOC(false);
    private final VoltageOut hoodVoltageRequest = new VoltageOut(0);

    // State
    private double targetSpeedRPS = 0.0;
    private double nominalTargetSpeedRPS = 0.0;
    private double targetHoodAngleRotations = kMinHoodAngleRotations; // Launch angle (mechanism rotations)
    private boolean isFiring = false;
    private boolean needsHoming = true;
    private boolean overrevActive = false;
    private double overrevRampStartTimestampSec = Double.NEGATIVE_INFINITY;
    private ShooterCharacterizationSession characterizationSession;
    private ShooterCharacterizationCsvLogger characterizationLogger;
    private ShooterCharacterizationStatus characterizationStatus = ShooterCharacterizationStatus.idle();
    private double characterizationOverCurrentStartSec = Double.NaN;
    private double cachedHoodKP, cachedHoodKI, cachedHoodKD, cachedHoodKV, cachedHoodKG;

    // Simulation context
    private Supplier<Pose2d> poseSupplier = Pose2d::new;
    private Supplier<ChassisSpeeds> speedsSupplier = ChassisSpeeds::new;
    private final TelemetryManager telemetry = TelemetryManager.getInstance();

    private static final int STATE_HOMING = 0;
    private static final int STATE_IDLE = 1;
    private static final int STATE_SPINNING_UP = 2;
    private static final int STATE_FIRING = 3;

    /** Clamp hood rotations to valid launch-angle range. */
    private static double clampHoodRotations(double rotations) {
        double minRot = ShooterConstants.launchRadToMechanismRotations(
                ShooterConstants.kMinHoodAngle.in(edu.wpi.first.units.Units.Radians));
        double maxRot = ShooterConstants.launchRadToMechanismRotations(
                ShooterConstants.kMaxHoodAngle.in(edu.wpi.first.units.Units.Radians));
        return Math.max(minRot, Math.min(maxRot, rotations));
    }

    public Shooter() {
        // Configure motors
        TalonUtil.applyAndCheckConfiguration(flywheel1, ShooterConstants.kFlywheelConfig);
        TalonUtil.applyAndCheckConfiguration(flywheel2, ShooterConstants.kFlywheelConfig);
        TalonUtil.applyAndCheckConfiguration(flywheel3, ShooterConstants.kFlywheelInvertedConfig);
        TalonUtil.applyAndCheckConfiguration(flywheel4, ShooterConstants.kFlywheelInvertedConfig);
        TalonUtil.applyAndCheckConfiguration(hoodMotor, ShooterConstants.kHoodConfig);
        initSmartDashboardGains();
        BaseStatusSignal.setUpdateFrequencyForAll(100.0, characterizationSignals);
        flywheel1.optimizeBusUtilization();
        flywheel2.optimizeBusUtilization();
        flywheel3.optimizeBusUtilization();
        flywheel4.optimizeBusUtilization();

        // Initialize simulation if needed
        if (Robot.isSimulation()) {
            initSimulation();
        }
    }

    private void initSimulation() {
        ProjectileSimulation.getInstance().setContext(
                poseSupplier,
                speedsSupplier,
                () -> flywheel1.getSimState().getMotorVoltage(),
                () -> hoodMotor.getSimState().getMotorVoltage());
    }

    /**
     * Set robot context for aiming calculations.
     */
    public void setSimulationContext(Supplier<Pose2d> pose, Supplier<ChassisSpeeds> speeds) {
        this.poseSupplier = pose;
        this.speedsSupplier = speeds;
        if (Robot.isSimulation()) {
            initSimulation();
        }
    }

    /**
     * Set target shooting solution from AutoAim.
     */
    public void setSetpoint(ShootingSolution solution) {
        double requestedSpeedRPS = solution.flywheelRpm() / 60.0;
        if (requestedSpeedRPS > kNonZeroSpeedEpsilonRPS && nominalTargetSpeedRPS <= kNonZeroSpeedEpsilonRPS) {
            startOverrev();
        } else if (requestedSpeedRPS <= kNonZeroSpeedEpsilonRPS) {
            clearOverrev();
        }

        // Flywheel uses direct map RPM command.
        this.nominalTargetSpeedRPS = requestedSpeedRPS;
        this.targetSpeedRPS = applyOverrev(requestedSpeedRPS);

        // Hood map uses commanded hood plane angle in degrees.
        double launchRad = Math.toRadians(solution.hoodCommandDeg());
        this.targetHoodAngleRotations = clampHoodRotations(
                ShooterConstants.launchRadToMechanismRotations(launchRad));
    }

    /**
     * Set flywheel speed directly (RPS).
     */
    public void setFlywheelSpeed(double rps) {
        clearOverrev();
        this.nominalTargetSpeedRPS = rps;
        this.targetSpeedRPS = rps;
    }

    /**
     * Set hood angle directly (rotations).
     */
    public void setHoodAngle(double rotations) {
        this.targetHoodAngleRotations = clampHoodRotations(rotations);
    }

    /**
     * Enable/disable firing mode.
     */
    public void setFiring(boolean firing) {
        this.isFiring = firing;
    }

    public boolean isFiring() {
        return isFiring;
    }

    /**
     * Stop all motors.
     */
    public void stop() {
        stopCharacterization();
        targetSpeedRPS = 0.0;
        nominalTargetSpeedRPS = 0.0;
        targetHoodAngleRotations = kMinHoodAngleRotations;
        isFiring = false;
        clearOverrev();
    }

    /**
     * Request hood re-homing against its minimum-angle hardstop.
     */
    public void requestHome() {
        stopCharacterization();
        targetSpeedRPS = 0.0;
        nominalTargetSpeedRPS = 0.0;
        targetHoodAngleRotations = kMinHoodAngleRotations;
        isFiring = false;
        needsHoming = true;
        clearOverrev();
    }

    /**
     * Check if shooter is at setpoint (ready to fire).
     */
    public boolean isAtSetpoint() {
        double currentRPS = getCurrentFlywheelRPS();
        double rpsError = Math.abs(currentRPS - targetSpeedRPS);

        double currentHoodRot = getCurrentHoodRotations();
        double hoodError = Math.abs(currentHoodRot - targetHoodAngleRotations);

        boolean flywheelReady = rpsError < kFlywheelSetpointToleranceRPS;
        boolean hoodReady = hoodError < kHoodSetpointToleranceRotations;

        return flywheelReady && hoodReady;
    }

    public double getCurrentFlywheelRPS() {
        if (Robot.isSimulation()) {
            return ProjectileSimulation.getInstance().getFlywheelVelocityRPS();
        }
        return flywheelVelocitySignal.getValueAsDouble();
    }

    public double getCurrentHoodRotations() {
        if (Robot.isSimulation()) {
            // Sim returns launch angle in rad.
            double launchRad = ProjectileSimulation.getInstance().getHoodAngleRad();
            return ShooterConstants.launchRadToMechanismRotations(launchRad);
        }
        return hoodMotor.getPosition().getValueAsDouble();
    }

    private void startOverrev() {
        overrevActive = true;
        overrevRampStartTimestampSec = Double.NEGATIVE_INFINITY;
    }

    private void clearOverrev() {
        overrevActive = false;
        overrevRampStartTimestampSec = Double.NEGATIVE_INFINITY;
    }

    private double applyOverrev(double requestedSpeedRPS) {
        if (requestedSpeedRPS <= kNonZeroSpeedEpsilonRPS) {
            return 0.0;
        }
        if (!overrevActive) {
            return requestedSpeedRPS;
        }

        double nowSec = Timer.getFPGATimestamp();

        if (overrevRampStartTimestampSec == Double.NEGATIVE_INFINITY) {
            double boostedTargetRPS = requestedSpeedRPS + kOverrevOffsetRPS;
            double speedErrorRPS = Math.abs(getCurrentFlywheelRPS() - boostedTargetRPS);
            if (speedErrorRPS < kFlywheelSetpointToleranceRPS) {
                overrevRampStartTimestampSec = nowSec;
            } else {
                return boostedTargetRPS;
            }
        }

        double elapsedSec = nowSec - overrevRampStartTimestampSec;
        double rampProgress = Math.max(0.0, Math.min(1.0, elapsedSec / kOverrevRampSeconds));
        double offsetRPS = kOverrevOffsetRPS * (1.0 - rampProgress);

        if (rampProgress >= 1.0) {
            clearOverrev();
            return requestedSpeedRPS;
        }

        return requestedSpeedRPS + offsetRPS;
    }

    private double getClosedLoopFlywheelCommandRps() {
        if (needsHoming) {
            return 0.0;
        }
        return targetSpeedRPS > kNonZeroSpeedEpsilonRPS ? targetSpeedRPS : kRestingFlywheelRPS;
    }

    public void startCharacterization(ShooterCharacterizationConfig config, ShooterCharacterizationMode mode) {
        stopCharacterization();
        if (mode == ShooterCharacterizationMode.DISABLED) {
            return;
        }
        if (!DriverStation.isTestEnabled()) {
            publishCharacterizationFailure(mode, "Characterization requires Test mode");
            return;
        }
        if (needsHoming) {
            publishCharacterizationFailure(mode, "Hood must be homed before characterization");
            return;
        }

        clearOverrev();
        isFiring = false;
        nominalTargetSpeedRPS = 0.0;
        targetSpeedRPS = 0.0;
        targetHoodAngleRotations = kMinHoodAngleRotations;

        int runId = kCharacterizationRunIds.getAndIncrement();
        characterizationSession = new ShooterCharacterizationSession(
                runId,
                mode,
                config,
                Timer.getFPGATimestamp());
        characterizationOverCurrentStartSec = Double.NaN;
        characterizationStatus = characterizationSession.status();
        publishCharacterizationStatus();
        try {
            characterizationLogger = ShooterCharacterizationCsvLogger.create(runId, mode);
        } catch (IOException e) {
            publishCharacterizationFailure(mode, "Failed to create log file: " + e.getMessage());
            characterizationSession = null;
            closeCharacterizationLogger();
        }
    }

    public void stopCharacterization() {
        if (characterizationSession != null && characterizationSession.isActive()) {
            characterizationSession.abort("Stopped");
            characterizationStatus = characterizationSession.status();
        }
        characterizationOverCurrentStartSec = Double.NaN;
        characterizationSession = null;
        closeCharacterizationLogger();
        publishCharacterizationStatus();
    }

    public boolean isCharacterizing() {
        return characterizationSession != null && characterizationSession.isActive();
    }

    public ShooterCharacterizationStatus getCharacterizationStatus() {
        return characterizationStatus;
    }

    private void initSmartDashboardGains() {
        cachedHoodKP = ShooterConstants.kHoodConfig.Slot0.kP;
        cachedHoodKI = ShooterConstants.kHoodConfig.Slot0.kI;
        cachedHoodKD = ShooterConstants.kHoodConfig.Slot0.kD;
        cachedHoodKV = ShooterConstants.kHoodConfig.Slot0.kV;
        cachedHoodKG = ShooterConstants.kHoodConfig.Slot0.kG;

        SmartDashboard.putNumber("Hood/kP", cachedHoodKP);
        SmartDashboard.putNumber("Hood/kI", cachedHoodKI);
        SmartDashboard.putNumber("Hood/kD", cachedHoodKD);
        SmartDashboard.putNumber("Hood/kV", cachedHoodKV);
        SmartDashboard.putNumber("Hood/kG", cachedHoodKG);
    }

    private void updateSmartDashboardGains() {
        double hoodKP = SmartDashboard.getNumber("Hood/kP", cachedHoodKP);
        double hoodKI = SmartDashboard.getNumber("Hood/kI", cachedHoodKI);
        double hoodKD = SmartDashboard.getNumber("Hood/kD", cachedHoodKD);
        double hoodKV = SmartDashboard.getNumber("Hood/kV", cachedHoodKV);
        double hoodKG = SmartDashboard.getNumber("Hood/kG", cachedHoodKG);

        if (hoodKP != cachedHoodKP || hoodKI != cachedHoodKI || hoodKD != cachedHoodKD
                || hoodKV != cachedHoodKV || hoodKG != cachedHoodKG) {
            cachedHoodKP = hoodKP;
            cachedHoodKI = hoodKI;
            cachedHoodKD = hoodKD;
            cachedHoodKV = hoodKV;
            cachedHoodKG = hoodKG;

            Slot0Configs hoodGains = new Slot0Configs()
                    .withKP(hoodKP)
                    .withKI(hoodKI)
                    .withKD(hoodKD)
                    .withKV(hoodKV)
                    .withKG(hoodKG);
            hoodMotor.getConfigurator().apply(hoodGains);
        }
    }

    @Override
    public void periodic() {
        updateSmartDashboardGains();

        if (!Robot.isSimulation()) {
            BaseStatusSignal.refreshAll(characterizationSignals);
        }

        double currentRPS = getCurrentFlywheelRPS();
        double currentHoodRot = getCurrentHoodRotations();
        double currentLaunchRad = ShooterConstants.mechanismRotationsToLaunchRad(currentHoodRot);
        double batteryVolts = RobotController.getBatteryVoltage();
        double avgFlywheelVoltage = average(
                flywheel1MotorVoltageSignal.getValueAsDouble(),
                flywheel2MotorVoltageSignal.getValueAsDouble(),
                flywheel3MotorVoltageSignal.getValueAsDouble(),
                flywheel4MotorVoltageSignal.getValueAsDouble());
        double totalFlywheelSupplyCurrent = sum(
                flywheel1SupplyCurrentSignal.getValueAsDouble(),
                flywheel2SupplyCurrentSignal.getValueAsDouble(),
                flywheel3SupplyCurrentSignal.getValueAsDouble(),
                flywheel4SupplyCurrentSignal.getValueAsDouble());
        double avgFlywheelSupplyCurrent = totalFlywheelSupplyCurrent / 4.0;
        double avgFlywheelStatorCurrent = average(
                flywheel1StatorCurrentSignal.getValueAsDouble(),
                flywheel2StatorCurrentSignal.getValueAsDouble(),
                flywheel3StatorCurrentSignal.getValueAsDouble(),
                flywheel4StatorCurrentSignal.getValueAsDouble());
        double electricalPowerWatts = batteryVolts * totalFlywheelSupplyCurrent;

        ShooterCharacterizationSession.Sample characterizationSample = null;
        boolean characterizationOpenLoop = false;
        double commandedFlywheelRps = getClosedLoopFlywheelCommandRps();
        double commandedFlywheelVolts = 0.0;
        if (characterizationSession != null) {
            if (!DriverStation.isTestEnabled()) {
                characterizationSession.abort("Test mode exited");
            } else if (batteryVolts < kCharacterizationMinBatteryVolts) {
                characterizationSession.abort("Battery below characterization threshold");
            } else if (totalFlywheelSupplyCurrent > kCharacterizationMaxCurrentAmps) {
                if (!Double.isFinite(characterizationOverCurrentStartSec)) {
                    characterizationOverCurrentStartSec = Timer.getFPGATimestamp();
                } else if (Timer.getFPGATimestamp() - characterizationOverCurrentStartSec
                        >= kCharacterizationCurrentDebounceSec) {
                    characterizationSession.abort("Flywheel current exceeded threshold");
                }
            } else if ((currentRPS * 60.0) > kCharacterizationMaxFlywheelRpm) {
                characterizationOverCurrentStartSec = Double.NaN;
                characterizationSession.abort("Flywheel RPM exceeded threshold");
            } else {
                characterizationOverCurrentStartSec = Double.NaN;
            }

            boolean atCurrentCharacterizationSetpoint = isAtCommandedSetpoint(
                    currentRPS,
                    currentHoodRot,
                    characterizationStatus.commandedRpm() / 60.0,
                    kMinHoodAngleRotations);
            characterizationSample = characterizationSession.update(Timer.getFPGATimestamp(), atCurrentCharacterizationSetpoint);
            commandedFlywheelRps = characterizationSample.commandedRpm() / 60.0;
            commandedFlywheelVolts = characterizationSample.commandedVolts();
            characterizationOpenLoop = characterizationSample.mode() == ShooterCharacterizationMode.OPEN_LOOP_VOLTAGE_SWEEP;
        }

        // === Flywheel control (always runs, independent of homing) ===
        if (characterizationOpenLoop) {
            flywheel1.setControl(flywheelVoltageRequest.withOutput(commandedFlywheelVolts));
            flywheel2.setControl(flywheelVoltageRequest.withOutput(commandedFlywheelVolts));
            flywheel3.setControl(flywheelVoltageRequest.withOutput(commandedFlywheelVolts));
            flywheel4.setControl(flywheelVoltageRequest.withOutput(commandedFlywheelVolts));
        } else {
            flywheel1.setControl(flywheelRequest.withVelocity(commandedFlywheelRps));
            flywheel2.setControl(flywheelRequest.withVelocity(commandedFlywheelRps));
            flywheel3.setControl(flywheelRequest.withVelocity(commandedFlywheelRps));
            flywheel4.setControl(flywheelRequest.withVelocity(commandedFlywheelRps));
        }

        // === Hood control (gated by homing) ===
        if (needsHoming) {
            // Drive hood toward min-angle hardstop
            hoodMotor.setControl(hoodVoltageRequest.withOutput(ShooterConstants.kHoodHomingVoltage));

            // Check for stall (hit hardstop): high current AND low velocity
            double current = hoodMotor.getStatorCurrent().getValueAsDouble();
            double velocity = Math.abs(hoodMotor.getVelocity().getValueAsDouble());
            if (current > ShooterConstants.kHoodStallCurrentThreshold && velocity < 0.5) {
                hoodMotor.setControl(hoodVoltageRequest.withOutput(0));
                hoodMotor.setPosition(ShooterConstants.launchRadToMechanismRotations(
                        ShooterConstants.kHoodHomePosition.in(edu.wpi.first.units.Units.Radians)));
                needsHoming = false;
            }
        } else {
            double hoodCommandRot = characterizationSession != null ? kMinHoodAngleRotations : targetHoodAngleRotations;
            hoodMotor.setControl(hoodRequest.withPosition(hoodCommandRot));
        }

        // === Telemetry (always runs) ===
        double targetHoodRot = characterizationSession != null ? kMinHoodAngleRotations : targetHoodAngleRotations;
        double targetLaunchRad = ShooterConstants.mechanismRotationsToLaunchRad(targetHoodRot);
        boolean atSetpoint = !needsHoming
                && isAtCommandedSetpoint(currentRPS, currentHoodRot, commandedFlywheelRps, targetHoodRot);
        int stateCode = needsHoming
                ? STATE_HOMING
                : (isFiring ? STATE_FIRING : (commandedFlywheelRps > 0.0 ? STATE_SPINNING_UP : STATE_IDLE));

        telemetry.publishShooterState(new ShooterSnapshot(
                needsHoming,
                isFiring,
                atSetpoint,
                stateCode,
                targetLaunchRad,
                targetLaunchRad,
                currentLaunchRad,
                targetLaunchRad - currentLaunchRad,
                hoodMotor.getVelocity().getValueAsDouble() * 2.0 * Math.PI,
                hoodMotor.getStatorCurrent().getValueAsDouble(),
                hoodMotor.getSupplyCurrent().getValueAsDouble(),
                hoodMotor.getMotorVoltage().getValueAsDouble(),
                commandedFlywheelRps,
                currentRPS,
                commandedFlywheelRps - currentRPS,
                avgFlywheelVoltage,
                avgFlywheelSupplyCurrent,
                avgFlywheelStatorCurrent));

        if (characterizationSession != null && characterizationSample != null) {
            characterizationStatus = characterizationSession.status();
            ShooterCharacterizationSnapshot snapshot = new ShooterCharacterizationSnapshot(
                    Timer.getFPGATimestamp(),
                    characterizationSample.runId(),
                    characterizationSample.mode().code(),
                    characterizationSample.segmentIndex(),
                    characterizationSample.commandedRpm(),
                    characterizationSample.commandedVolts(),
                    currentRPS * 60.0,
                    batteryVolts,
                    avgFlywheelVoltage,
                    totalFlywheelSupplyCurrent,
                    avgFlywheelStatorCurrent,
                    electricalPowerWatts,
                    characterizationSample.commandedRpm() - (currentRPS * 60.0),
                    atSetpoint,
                    characterizationSample.settled(),
                    characterizationSample.aborted());
            telemetry.publishShooterCharacterization(snapshot);
            publishCharacterizationStatus();
            tryAppendCharacterizationRow(characterizationSample.mode(), snapshot);

            if (!characterizationSample.active()) {
                characterizationOverCurrentStartSec = Double.NaN;
                characterizationSession = null;
                closeCharacterizationLogger();
                publishCharacterizationStatus();
            }
        }
    }

    @Override
    public void simulationPeriodic() {
        // Update projectile simulation
        ProjectileSimulation sim = ProjectileSimulation.getInstance();
        sim.update(0.02, isFiring, getClosedLoopFlywheelCommandRps(), targetHoodAngleRotations);

        // Feed simulated values back to motor sim states
        double flywheelRPS = sim.getFlywheelVelocityRPS();
        // Sim API needs rotor (motor) velocity, so we convert back to motor space
        flywheel1.getSimState().setRotorVelocity(flywheelRPS * ShooterConstants.kFlywheelGearRatio);

        // Sim returns launch angle in rad -> convert to motor rotor position
        double launchAngleRad = sim.getHoodAngleRad();
        double mechanismRot = ShooterConstants.launchRadToMechanismRotations(launchAngleRad);
        hoodMotor.getSimState().setRawRotorPosition(mechanismRot * ShooterConstants.kHoodGearRatio);

        double launchVelRadPerSec = sim.getHoodVelocityRadPerSec();
        double launchVelRot = Units.radiansToRotations(launchVelRadPerSec);
        hoodMotor.getSimState().setRotorVelocity(launchVelRot * ShooterConstants.kHoodGearRatio);

    }

    /**
     * Debug: Seed field with game pieces.
     */
    public void seedField() {
        if (Robot.isSimulation()) {
            ProjectileSimulation.getInstance().seedField();
        }
    }

    /**
     * Returns whether homing is complete.
     */
    public boolean isHomed() {
        return !needsHoming;
    }

    private boolean isAtCommandedSetpoint(
            double currentFlywheelRps,
            double currentHoodRot,
            double commandedFlywheelRps,
            double commandedHoodRot) {
        double rpsError = Math.abs(currentFlywheelRps - commandedFlywheelRps);
        double hoodError = Math.abs(currentHoodRot - commandedHoodRot);
        return rpsError < kFlywheelSetpointToleranceRPS && hoodError < kHoodSetpointToleranceRotations;
    }

    private void publishCharacterizationFailure(ShooterCharacterizationMode mode, String reason) {
        characterizationStatus = new ShooterCharacterizationStatus(
                false,
                false,
                true,
                0,
                mode,
                -1,
                0,
                0.0,
                0.0,
                reason);
        publishCharacterizationStatus();
    }

    private void publishCharacterizationStatus() {
        telemetry.publishShooterCharacterizationStatus(
                characterizationStatus.active(),
                characterizationStatus.complete(),
                characterizationStatus.aborted(),
                characterizationStatus.runId(),
                characterizationStatus.mode().code(),
                characterizationStatus.mode().name(),
                characterizationStatus.segmentIndex(),
                characterizationStatus.totalSegments(),
                characterizationStatus.abortReason());
    }

    private void closeCharacterizationLogger() {
        if (characterizationLogger == null) {
            return;
        }
        try {
            characterizationLogger.close();
        } catch (IOException e) {
            DriverStation.reportWarning("Failed to close shooter characterization log: " + e.getMessage(), false);
        }
        characterizationLogger = null;
    }

    private void tryAppendCharacterizationRow(
            ShooterCharacterizationMode mode,
            ShooterCharacterizationSnapshot snapshot) {
        if (characterizationLogger == null) {
            return;
        }
        try {
            characterizationLogger.append(mode, snapshot);
        } catch (IOException e) {
            if (characterizationSession != null) {
                characterizationSession.abort("Failed to write characterization log");
                characterizationStatus = characterizationSession.status();
            }
            DriverStation.reportWarning("Failed to append shooter characterization log: " + e.getMessage(), false);
            closeCharacterizationLogger();
        }
    }

    private static double average(double... values) {
        return sum(values) / values.length;
    }

    private static double sum(double... values) {
        double total = 0.0;
        for (double value : values) {
            total += value;
        }
        return total;
    }
}
