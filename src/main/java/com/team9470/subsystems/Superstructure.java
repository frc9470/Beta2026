package com.team9470.subsystems;

import com.team9470.TunerConstants;
import com.team9470.subsystems.hopper.Hopper;
import com.team9470.subsystems.intake.Intake;
import com.team9470.subsystems.shooter.Shooter;
import com.team9470.subsystems.shooter.ShooterConstants;
import com.team9470.telemetry.MatchTimingService;
import com.team9470.telemetry.TelemetryManager;
import com.team9470.telemetry.structs.HopperPreloadSnapshot;
import com.team9470.telemetry.structs.ShotReleaseSnapshot;
import com.team9470.telemetry.structs.SuperstructureSnapshot;
import com.team9470.telemetry.structs.TimedShotSnapshot;
import com.team9470.util.AutoAim;
import com.team9470.util.GeomUtil;
import com.team9470.util.TimedFirePolicy;

import com.team9470.subsystems.swerve.Swerve;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static edu.wpi.first.units.Units.Meters;

import java.util.concurrent.atomic.AtomicBoolean;
import java.util.function.Supplier;

/**
 * Superstructure manages all mechanisms (shooter, intake, hopper) and provides
 * high-level commands for coordinated behavior.
 */
public class Superstructure extends SubsystemBase {

    private static Superstructure instance;

    public static Superstructure getInstance() {
        if (instance == null) {
            instance = new Superstructure();
        }
        return instance;
    }

    // Subsystems
    private final Shooter shooter;
    private final Intake intake;
    private final Hopper hopper;
    private final MatchTimingService matchTimingService = MatchTimingService.getInstance();
    private final TelemetryManager telemetry = TelemetryManager.getInstance();

    // Context suppliers (set by RobotContainer)
    private Supplier<Pose2d> poseSupplier = () -> new Pose2d();
    private Supplier<ChassisSpeeds> speedsSupplier = () -> new ChassisSpeeds();
    private static final double kControlLoopDtSec = 0.02;
    private static final double kAimKp = 10.0;
    private static final double kAimKd = 0.7;
    private static final double kAimAlignmentToleranceRad = Math.toRadians(5.0);
    private static final double kAimLatchedAlignmentToleranceRad = Math.toRadians(15.0);
    private static final double kAimMaxAngularRateRadPerSec = Math.toRadians(TunerConstants.maxAngularVelocity);
    private static final double kDriveAngleDerivativeFilterWindowSec = 1.5;
    private static final double kHoodDerivativeFilterWindowSec = 0.4;
    private static final double kFlywheelDerivativeFilterWindowSec = 0.4;
    private static final double kShootMaxPolarVelocityRadPerSec = 0.5;
    private static final double kShootVelocityLimitMinRequestMps = 0.15;
    private static final double kSotmFireSafetyMinSpeedMps = 0.35;
    private static final double kSotmMaxAccelerationForFireMps2 = 3.5;
    private static final double kSotmMaxOmegaForFireRadPerSec = Math.toRadians(220.0);
    // COR shifting constants (ported from MechAdv DriveCommands)
    private static final double kCORMinErrorDeg = 15.0;
    private static final double kCORMaxErrorDeg = 30.0;
    // O-Lock thresholds (ported from MechAdv DriveCommands)
    private static final double kOLockSpeedThresholdMps = 0.1;
    private static final double kOLockOmegaThresholdRadPerSec = 0.15;
    private static final double kPreloadSettleVolts = -7.0;
    private static final double kPreloadSettleSec = 0.50;
    private static final double kPreloadStageHopperVolts = -3.5;
    private static final double kPreloadStageFeederVolts = -1.5;
    private static final double kPreloadTimeoutSec = 2.5;
    private static final double kPreloadJamCurrentAmps = 18.0;
    private static final double kPreloadJamVelocityRps = 1.0;
    private static final double kPreloadJamDebounceSec = 0.20;
    private static final int PRELOAD_PHASE_IDLE = 0;
    private static final int PRELOAD_PHASE_ALREADY_STAGED = 1;
    private static final int PRELOAD_PHASE_SETTLING = 2;
    private static final int PRELOAD_PHASE_STAGING = 3;
    private static final int PRELOAD_PHASE_DONE = 4;
    private static final int PRELOAD_FAULT_NONE = 0;
    private static final int PRELOAD_FAULT_TIMEOUT = 1;
    private static final int PRELOAD_FAULT_JAM = 2;

    /** Seconds before / after the active zone to keep the flywheel boosted. */
    private static final double kIdleBoostPaddingSec = 3.0;

    private LinearFilter driveAngleRateFilter = createMovingAverageFilter(kDriveAngleDerivativeFilterWindowSec);
    private LinearFilter hoodRateFilter = createMovingAverageFilter(kHoodDerivativeFilterWindowSec);
    private LinearFilter flywheelRateFilter = createMovingAverageFilter(kFlywheelDerivativeFilterWindowSec);
    private boolean derivativeStateInitialized = false;
    private Rotation2d lastTargetYaw = new Rotation2d();
    private double lastHoodCommandDeg = 0.0;
    private double lastFlywheelRpm = 0.0;
    private String lastReleaseBlockReason = "NoBlockRecordedYet";
    /** FPGA timestamp when the zone was last active, for the post-active cooldown. */
    private double lastZoneActiveSec = Double.NEGATIVE_INFINITY;

    private Superstructure() {
        shooter = new Shooter();
        intake = Intake.getInstance();
        hopper = Hopper.getInstance();
        publishPreloadState(false, PRELOAD_PHASE_IDLE, PRELOAD_FAULT_NONE);
        resetTimedShotState();
        telemetry.publishSuperstructureReleaseBlock(false, lastReleaseBlockReason);
    }

    /**
     * Set the pose and chassis speeds suppliers for aiming calculations.
     * Called by RobotContainer to connect swerve data.
     */
    public void setDriveContext(Supplier<Pose2d> pose, Supplier<ChassisSpeeds> speeds) {
        this.poseSupplier = pose;
        this.speedsSupplier = speeds;
        shooter.setSimulationContext(pose, speeds);
    }

    @Override
    public void periodic() {
        AutoAim.publishModeTelemetry(poseSupplier.get());

        // Let the shooter know whether the intake is running so it can
        // decide whether to idle-spin the flywheel.
        shooter.setIntakeRunning(intake.isRunning());

        // Boost flywheel idle speed around the active zone window
        updateActiveIdleBoost();
    }

    /**
     * Boost flywheel idle speed around the active zone:
     * - 3 s before the zone becomes active (inactive zone remaining ≤ 3 s)
     * - During the active zone
     * - 3 s after the zone ends
     */
    private void updateActiveIdleBoost() {
        double nowSec = Timer.getFPGATimestamp();
        boolean zoneActive = matchTimingService.zoneActive();

        if (zoneActive) {
            lastZoneActiveSec = nowSec;
            shooter.setActiveIdleBoost(true);
            return;
        }

        // Post-active cooldown: keep boosting for kIdleBoostPaddingSec after zone ended
        if (nowSec - lastZoneActiveSec <= kIdleBoostPaddingSec) {
            shooter.setActiveIdleBoost(true);
            return;
        }

        // Pre-active ramp: boost if the inactive zone ends within kIdleBoostPaddingSec
        if (matchTimingService.timingKnown()) {
            double remaining = matchTimingService.zoneRemainingSec();
            if (!zoneActive && remaining <= kIdleBoostPaddingSec) {
                shooter.setActiveIdleBoost(true);
                return;
            }
        }

        shooter.setActiveIdleBoost(false);
    }

    // ==================== HIGH-LEVEL COMMANDS ====================

    /**
     * Intake command - deploys intake and runs rollers while held.
     */
    public Command intakeCommand() {
        return intake.getIntakeCommand()
                .withName("Superstructure Intake");
    }

    /**
     * Toggle intake arm deployed/retracted.
     */
    public Command toggleIntakeCommand() {
        return intake.getToggleCommand()
                .withName("Superstructure Toggle Intake");
    }

    /**
     * Toggle intake arm to deploy-high/retracted.
     */
    public Command toggleIntakeHighCommand() {
        return intake.getDeployHighToggleCommand()
                .withName("Superstructure Toggle Intake High");
    }

    /**
     * Agitate intake while held.
     */
    public Command agitateIntakeCommand() {
        return intake.getAgitateCommand()
                .withName("Superstructure Agitate Intake");
    }

    /**
     * Outtake command - reverses rollers while held.
     */
    public Command outtakeCommand() {
        return intake.getOuttakeCommand()
                .withName("Superstructure Outtake");
    }

    /** Threshold in seconds: presses shorter than this are treated as a tap. */
    private static final double kManualIntakeTapThresholdSec = 0.30;

    /**
     * Manual intake control command (right bumper). Tap vs. hold is decided on
     * release so the two actions never overlap.
     * <ul>
     *   <li><b>Tap</b> (release before {@link #kManualIntakeTapThresholdSec}):
     *       stow the intake. Rollers are never overridden.</li>
     *   <li><b>Hold</b> (past the threshold): stop the rollers while held; the
     *       intake pivot stays where it was. No stow on release.</li>
     * </ul>
     */
    public Command manualIntakeControlCommand() {
        Timer holdTimer = new Timer();
        return Commands.runEnd(
                () -> {
                    if (holdTimer.get() > kManualIntakeTapThresholdSec) {
                        intake.setOverrideStopRollers(true);
                    }
                },
                () -> {
                    boolean wasTap = holdTimer.get() <= kManualIntakeTapThresholdSec;
                    intake.setOverrideStopRollers(false);
                    if (wasTap) {
                        intake.stow();
                    }
                    holdTimer.stop();
                },
                intake)
                .beforeStarting(holdTimer::restart)
                .withName("Superstructure Manual Intake Control");
    }

    /**
     * Feed command - spins shooter at 500 RPM with max hood angle and feeds game
     * pieces.
     * Intended for passing/feeding to a partner robot.
     */
    public Command feedCommand() {
        final double feedRps = 500.0 / 60.0; // 500 RPM -> RPS
        final double maxHoodRotations = com.team9470.subsystems.shooter.ShooterConstants
                .launchRadToMechanismRotations(
                        com.team9470.subsystems.shooter.ShooterConstants.kMaxHoodAngle.in(
                                edu.wpi.first.units.Units.Radians));
        return Commands.runEnd(
                () -> {
                    shooter.setFlywheelSpeed(feedRps);
                    shooter.setHoodAngle(maxHoodRotations);
                    shooter.setFiring(true);
                    hopper.setRunning(true);
                },
                () -> {
                    shooter.stop();
                    hopper.stop();
                }).withName("Superstructure Feed");
    }

    /**
     * Re-home both intake pivot and shooter hood.
     */
    public Command homeIntakeAndHoodCommand() {
        return Commands.runOnce(() -> {
            intake.requestHome();
            shooter.requestHome();
        }, intake, shooter).withName("Superstructure Home Intake+Hood");
    }

    public Command stagePreloadCommand() {
        return stagePreloadCommand(true);
    }

    /**
     * Auto-safe preload staging. This intentionally does not reserve the whole
     * superstructure, because sequential auto groups reserve all future child
     * requirements up front and would otherwise cancel themselves when this is
     * triggered mid-path.
     */
    public Command stagePreloadAutoCommand() {
        return stagePreloadCommand(false);
    }

    private Command stagePreloadCommand(boolean reserveSuperstructure) {
        final int[] phaseCode = { PRELOAD_PHASE_IDLE };
        final int[] faultCode = { PRELOAD_FAULT_NONE };
        final boolean[] finished = { false };
        final double[] commandStartSec = { Double.NaN };
        final double[] phaseStartSec = { Double.NaN };
        final double[] jamStartSec = { Double.NaN };
        Subsystem[] requirements = reserveSuperstructure
                ? new Subsystem[] { this, hopper }
                : new Subsystem[] { hopper };

        return Commands.run(() -> {
            double nowSec = Timer.getFPGATimestamp();
            if (phaseCode[0] != PRELOAD_PHASE_ALREADY_STAGED && hopper.isTopBeamBreakBlocked()) {
                phaseCode[0] = PRELOAD_PHASE_DONE;
                finished[0] = true;
                publishPreloadState(true, phaseCode[0], faultCode[0]);
                return;
            }
            if (nowSec - commandStartSec[0] >= kPreloadTimeoutSec) {
                faultCode[0] = PRELOAD_FAULT_TIMEOUT;
                finished[0] = true;
                publishPreloadState(true, phaseCode[0], faultCode[0]);
                return;
            }

            if (phaseCode[0] == PRELOAD_PHASE_SETTLING) {
                hopper.setHopperVoltage(kPreloadSettleVolts);
                hopper.setFeederVoltage(0.0);
                if (nowSec - phaseStartSec[0] >= kPreloadSettleSec) {
                    phaseCode[0] = PRELOAD_PHASE_STAGING;
                    phaseStartSec[0] = nowSec;
                }
            } else if (phaseCode[0] == PRELOAD_PHASE_STAGING) {
                hopper.setHopperVoltage(kPreloadStageHopperVolts);
                hopper.setFeederVoltage(kPreloadStageFeederVolts);

                double averageFeederCurrentAmps = (Math.abs(hopper.getFeederLeftStatorCurrentAmps())
                        + Math.abs(hopper.getFeederRightStatorCurrentAmps()))
                        * 0.5;
                double averageFeederVelocityRps = (Math.abs(hopper.getFeederLeftVelocityRps())
                        + Math.abs(hopper.getFeederRightVelocityRps()))
                        * 0.5;
                boolean jamDetected = averageFeederCurrentAmps > kPreloadJamCurrentAmps
                        && averageFeederVelocityRps < kPreloadJamVelocityRps;
                if (jamDetected) {
                    if (!Double.isFinite(jamStartSec[0])) {
                        jamStartSec[0] = nowSec;
                    } else if (nowSec - jamStartSec[0] >= kPreloadJamDebounceSec) {
                        faultCode[0] = PRELOAD_FAULT_JAM;
                        finished[0] = true;
                    }
                } else {
                    jamStartSec[0] = Double.NaN;
                }
            }
            publishPreloadState(true, phaseCode[0], faultCode[0]);
        }, requirements).beforeStarting(() -> {
            double nowSec = Timer.getFPGATimestamp();
            finished[0] = false;
            faultCode[0] = PRELOAD_FAULT_NONE;
            commandStartSec[0] = nowSec;
            phaseStartSec[0] = nowSec;
            jamStartSec[0] = Double.NaN;
            hopper.stopAll();

            if (hopper.isTopBeamBreakBlocked()) {
                phaseCode[0] = PRELOAD_PHASE_ALREADY_STAGED;
                finished[0] = true;
            } else {
                phaseCode[0] = PRELOAD_PHASE_SETTLING;
            }
            publishPreloadState(true, phaseCode[0], faultCode[0]);
        }).until(() -> finished[0]).finallyDo(() -> {
            hopper.stopAll();
            publishPreloadState(false, phaseCode[0], faultCode[0]);
        }).withName("Superstructure StagePreload");
    }

    /**
     * Aim and shoot command - calculates aim and returns rotation command for
     * swerve.
     * Use with swerve to get the rotation rate.
     */
    public AimResult getAimResult() {
        return getAimResult(false);
    }

    public AimResult getAimResult(boolean useRobotSideForFeedTarget) {
        Pose2d robotPose = poseSupplier.get();
        ChassisSpeeds robotSpeeds = speedsSupplier.get();
        return getAimResult(robotPose, robotSpeeds, useRobotSideForFeedTarget);
    }

    public AimResult getAimResult(Pose2d robotPose, ChassisSpeeds robotSpeeds) {
        return getAimResult(robotPose, robotSpeeds, false);
    }

    public AimResult getAimResult(Pose2d robotPose, ChassisSpeeds robotSpeeds, boolean useRobotSideForFeedTarget) {
        var solution = AutoAim.calculate(robotPose, robotSpeeds, useRobotSideForFeedTarget);
        AimSetpointDerivatives derivatives = computeFilteredSetpointDerivatives(solution);
        double rotError = solution.targetRobotYaw()
                .minus(robotPose.getRotation())
                .getRadians();
        double driveSetpointRateRadPerSec = derivatives.driveAngleRateRadPerSec();
        double rotCmd = driveSetpointRateRadPerSec
                + (rotError * kAimKp)
                + ((driveSetpointRateRadPerSec - robotSpeeds.omegaRadiansPerSecond) * kAimKd);
        rotCmd = MathUtil.clamp(rotCmd, -kAimMaxAngularRateRadPerSec, kAimMaxAngularRateRadPerSec);
        boolean isAligned = Math.abs(rotError) < kAimAlignmentToleranceRad;

        return new AimResult(rotCmd, isAligned, rotError, derivatives, solution);
    }

    /**
     * Result of aiming calculation for swerve to use.
     */
    public record AimSetpointDerivatives(
            double driveAngleRateRadPerSec,
            double hoodRateDegPerSec,
            double flywheelRateRpmPerSec) {
    }

    public record AimResult(
            double rotationCommand,
            boolean isAligned,
            double rotationErrorRad,
            AimSetpointDerivatives derivatives,
            AutoAim.ShootingSolution solution) {
    }

    private record ReleaseTelemetryInputs(
            boolean canFire,
            boolean releaseAllowed,
            boolean releaseStartedThisHold,
            boolean shotValid,
            boolean hoodAtSetpoint,
            boolean flywheelAtSetpoint,
            boolean flywheelReadyForRelease,
            boolean aligned,
            double alignmentToleranceRad,
            double rotationErrorRad,
            boolean sotmFireSafe,
            ChassisSpeeds robotSpeeds,
            AutoAim.ShootingSolution solution,
            boolean timedAutoArmCandidate,
            boolean armedDuringInactiveThisHold,
            boolean timedReleaseStarted,
            boolean timingKnown,
            boolean zoneActive,
            boolean topSensorBlocked,
            boolean releaseWindowOpen,
            double zoneRemainingSec,
            double launchLeadSec,
            boolean timedReleaseAllowed,
            boolean shotIsFeedMode) {
    }

    private static LinearFilter createMovingAverageFilter(double windowSec) {
        int taps = Math.max(1, (int) Math.round(windowSec / kControlLoopDtSec));
        return LinearFilter.movingAverage(taps);
    }

    private void resetAimSetpointDerivatives() {
        driveAngleRateFilter = createMovingAverageFilter(kDriveAngleDerivativeFilterWindowSec);
        hoodRateFilter = createMovingAverageFilter(kHoodDerivativeFilterWindowSec);
        flywheelRateFilter = createMovingAverageFilter(kFlywheelDerivativeFilterWindowSec);
        derivativeStateInitialized = false;
        lastTargetYaw = new Rotation2d();
        lastHoodCommandDeg = 0.0;
        lastFlywheelRpm = 0.0;
    }

    private AimSetpointDerivatives computeFilteredSetpointDerivatives(AutoAim.ShootingSolution solution) {
        if (!derivativeStateInitialized) {
            derivativeStateInitialized = true;
            lastTargetYaw = solution.targetRobotYaw();
            lastHoodCommandDeg = solution.hoodCommandDeg();
            lastFlywheelRpm = solution.flywheelRpm();
            return new AimSetpointDerivatives(0.0, 0.0, 0.0);
        }

        double rawDriveAngleRateRadPerSec = solution.targetRobotYaw().minus(lastTargetYaw).getRadians()
                / kControlLoopDtSec;
        double rawHoodRateDegPerSec = (solution.hoodCommandDeg() - lastHoodCommandDeg) / kControlLoopDtSec;
        double rawFlywheelRateRpmPerSec = (solution.flywheelRpm() - lastFlywheelRpm) / kControlLoopDtSec;

        double filteredDriveAngleRateRadPerSec = driveAngleRateFilter.calculate(rawDriveAngleRateRadPerSec);
        double filteredHoodRateDegPerSec = hoodRateFilter.calculate(rawHoodRateDegPerSec);
        double filteredFlywheelRateRpmPerSec = flywheelRateFilter.calculate(rawFlywheelRateRpmPerSec);

        lastTargetYaw = solution.targetRobotYaw();
        lastHoodCommandDeg = solution.hoodCommandDeg();
        lastFlywheelRpm = solution.flywheelRpm();

        return new AimSetpointDerivatives(
                filteredDriveAngleRateRadPerSec,
                filteredHoodRateDegPerSec,
                filteredFlywheelRateRpmPerSec);
    }

    private static Translation2d limitTranslationForLaunch(
            Pose2d robotPose,
            AutoAim.ShootingSolution solution,
            double requestedVxMps,
            double requestedVyMps) {
        Translation2d requestedVelocity = new Translation2d(requestedVxMps, requestedVyMps);
        if (!AutoAim.isSotmEnabled()) {
            return requestedVelocity;
        }
        double requestedSpeed = requestedVelocity.getNorm();
        if (requestedSpeed < kShootVelocityLimitMinRequestMps) {
            return requestedVelocity;
        }
        if (AutoAim.isFeedModeActive(robotPose) || !solution.isValid()) {
            return requestedVelocity;
        }

        double naiveAirTimeSec = solution.naiveAirTimeSec();
        double distanceNoLookaheadMeters = solution.distanceNoLookaheadMeters();
        if (!Double.isFinite(naiveAirTimeSec)
                || !Double.isFinite(distanceNoLookaheadMeters)
                || naiveAirTimeSec <= 1e-3
                || distanceNoLookaheadMeters <= 1e-3) {
            return requestedVelocity;
        }

        Translation2d robotToTarget = AutoAim.getTarget(robotPose).toTranslation2d().minus(robotPose.getTranslation());
        if (robotToTarget.getNorm() <= 1e-3) {
            return requestedVelocity;
        }

        double hubAngle = kShootMaxPolarVelocityRadPerSec * naiveAirTimeSec;
        if (hubAngle <= 1e-6 || hubAngle >= Math.PI - 1e-3) {
            return requestedVelocity;
        }

        double robotAngle = Math.abs(robotToTarget.getAngle().minus(requestedVelocity.getAngle()).getRadians());
        double lookaheadAngle = Math.PI - robotAngle - hubAngle;
        if (lookaheadAngle <= 1e-3) {
            return requestedVelocity;
        }

        double lookaheadSin = Math.sin(lookaheadAngle);
        if (Math.abs(lookaheadSin) <= 1e-4) {
            return requestedVelocity;
        }

        double robotLookaheadDistance = distanceNoLookaheadMeters * Math.sin(hubAngle) / lookaheadSin;
        if (!Double.isFinite(robotLookaheadDistance) || robotLookaheadDistance <= 0.0) {
            return requestedVelocity;
        }

        double maxLinearSpeedMps = robotLookaheadDistance / naiveAirTimeSec;
        if (!Double.isFinite(maxLinearSpeedMps) || maxLinearSpeedMps <= 0.0 || requestedSpeed <= maxLinearSpeedMps) {
            return requestedVelocity;
        }
        return requestedVelocity.times(maxLinearSpeedMps / requestedSpeed);
    }

    private static boolean isSotmFireSafe(AutoAim.ShootingSolution solution, ChassisSpeeds robotSpeeds) {
        if (!AutoAim.isSotmEnabled()) {
            return true;
        }
        if (solution.shooterFieldSpeedMps() < kSotmFireSafetyMinSpeedMps) {
            // Stationary or near-stationary shot path: don't apply moving-shot inhibit.
            return true;
        }
        if (solution.shooterFieldAccelMps2() > kSotmMaxAccelerationForFireMps2) {
            return false;
        }
        if (Math.abs(robotSpeeds.omegaRadiansPerSecond) > kSotmMaxOmegaForFireRadPerSec) {
            return false;
        }
        return true;
    }

    static boolean shouldAllowRelease(
            boolean canFire,
            boolean releaseStartedThisHold) {
        // Once release has started, keep feeding until the command ends.
        if (releaseStartedThisHold) {
            return true;
        }
        return canFire;
    }

    static boolean shouldTreatFlywheelAsReady(
            boolean flywheelAtSetpoint,
            boolean releaseStartedThisHold,
            boolean flywheelReadyLatchedThisHold) {
        if (!releaseStartedThisHold) {
            return flywheelAtSetpoint;
        }
        return flywheelReadyLatchedThisHold;
    }

    static boolean shouldTreatAlignmentAsReady(
            boolean alignedAtTightTolerance,
            boolean alignmentReadyLatchedThisHold,
            double rotationErrorRad) {
        if (alignedAtTightTolerance) {
            return true;
        }
        return alignmentReadyLatchedThisHold && Math.abs(rotationErrorRad) < kAimLatchedAlignmentToleranceRad;
    }

    private String describeShooterReadinessBlock(boolean hoodAtSetpoint, boolean flywheelAtSetpoint) {
        if (!shooter.isHomed()) {
            return "HoodNotHomed";
        }
        if (!hoodAtSetpoint && !flywheelAtSetpoint) {
            return "HoodAndFlywheelNotReady";
        }
        if (!hoodAtSetpoint) {
            return "HoodNotReady";
        }
        if (!flywheelAtSetpoint) {
            return "FlywheelNotReady";
        }
        return "ShooterNotReady";
    }

    private String determineLiveReleaseBlockReason(
            boolean shotValid,
            boolean hoodAtSetpoint,
            boolean flywheelAtSetpoint,
            boolean aligned,
            boolean sotmFireSafe) {
        if (!shotValid) {
            return "ShotInvalid";
        }
        if (!hoodAtSetpoint || !flywheelAtSetpoint) {
            return describeShooterReadinessBlock(hoodAtSetpoint, flywheelAtSetpoint);
        }
        if (!aligned) {
            return "NotAligned";
        }
        if (!sotmFireSafe) {
            return "SotmUnsafe";
        }
        return "ReleaseAllowed";
    }

    private String determineReleaseBlockReason(
            TimedFirePolicy.TimedFireDecision timedFireDecision,
            boolean timedReleaseStartedThisHold,
            boolean shotValid,
            boolean hoodAtSetpoint,
            boolean flywheelAtSetpoint,
            boolean aligned,
            boolean sotmFireSafe) {
        if (!timedFireDecision.timedAutoArmCandidate() || timedReleaseStartedThisHold
                || timedFireDecision.allowFeed()) {
            return determineLiveReleaseBlockReason(shotValid, hoodAtSetpoint, flywheelAtSetpoint, aligned,
                    sotmFireSafe);
        }
        return switch (timedFireDecision.reasonCode()) {
            case TimedFirePolicy.REASON_SHOOTER_NOT_READY ->
                describeShooterReadinessBlock(hoodAtSetpoint, flywheelAtSetpoint);
            case TimedFirePolicy.REASON_NOT_ALIGNED -> "NotAligned";
            case TimedFirePolicy.REASON_SHOT_INVALID -> "ShotInvalid";
            default -> TimedFirePolicy.reasonLabel(timedFireDecision.reasonCode());
        };
    }

    private void publishReleaseBlockTelemetry(boolean blocked, String currentReason) {
        if (blocked && currentReason != null && !currentReason.isBlank()) {
            lastReleaseBlockReason = currentReason;
        }
        telemetry.publishSuperstructureReleaseBlock(blocked, lastReleaseBlockReason);
    }

    private void publishReleaseConditionTelemetry(ReleaseTelemetryInputs inputs) {
        boolean sotmSafetyActive = inputs.solution().shooterFieldSpeedMps() >= kSotmFireSafetyMinSpeedMps;
        telemetry.publishSuperstructureReleaseState(new ShotReleaseSnapshot(
                inputs.canFire(),
                inputs.releaseAllowed(),
                inputs.releaseStartedThisHold(),
                inputs.shotValid(),
                shooter.isHomed(),
                inputs.hoodAtSetpoint(),
                Math.toDegrees(shooter.getHoodErrorRad()),
                Math.toDegrees(shooter.getHoodSetpointToleranceRad()),
                inputs.flywheelAtSetpoint(),
                inputs.flywheelReadyForRelease(),
                shooter.getFlywheelErrorRps(),
                shooter.getFlywheelSetpointToleranceRps(),
                inputs.aligned(),
                Math.toDegrees(inputs.rotationErrorRad()),
                Math.toDegrees(inputs.alignmentToleranceRad()),
                inputs.sotmFireSafe(),
                sotmSafetyActive,
                inputs.solution().shooterFieldSpeedMps(),
                inputs.solution().shooterFieldAccelMps2(),
                kSotmMaxAccelerationForFireMps2,
                Math.toDegrees(inputs.robotSpeeds().omegaRadiansPerSecond),
                Math.toDegrees(kSotmMaxOmegaForFireRadPerSec),
                inputs.timedAutoArmCandidate(),
                inputs.armedDuringInactiveThisHold(),
                inputs.timedReleaseStarted(),
                inputs.timingKnown(),
                inputs.zoneActive(),
                inputs.topSensorBlocked(),
                inputs.releaseWindowOpen(),
                inputs.zoneRemainingSec(),
                inputs.launchLeadSec(),
                inputs.timedReleaseAllowed(),
                inputs.shotIsFeedMode()));
    }

    /**
     * Aim, rotate, and shoot — auto version (robot stationary, only rotates to
     * aim).
     */
    public Command aimAndShootCommand() {
        return aimAndShootCommand(() -> 0.0, () -> 0.0, false);
    }

    /**
     * Aim, rotate, and shoot — teleop version (driver controls translation).
     * Handles swerve rotation automatically via auto-aim.
     *
     * @param vxSupplier field-relative X velocity (m/s)
     * @param vySupplier field-relative Y velocity (m/s)
     */
    public Command aimAndShootCommand(Supplier<Double> vxSupplier, Supplier<Double> vySupplier) {
        return aimAndShootCommand(vxSupplier, vySupplier, false);
    }

    /**
     * Aim, rotate, and shoot — full version with explicit feed-side targeting
     * selection.
     * Handles swerve rotation automatically via auto-aim.
     *
     * @param vxSupplier                field-relative X velocity (m/s)
     * @param vySupplier                field-relative Y velocity (m/s)
     * @param useRobotSideForFeedTarget when true, feed mode target side is
     *                                  selected from robot side (left/right)
     */
    public Command aimAndShootCommand(
            Supplier<Double> vxSupplier,
            Supplier<Double> vySupplier,
            boolean useRobotSideForFeedTarget) {
        Swerve swerve = Swerve.getInstance();
        AtomicBoolean armedDuringInactiveThisHold = new AtomicBoolean(false);
        AtomicBoolean timedReleaseStartedThisHold = new AtomicBoolean(false);
        AtomicBoolean releaseStartedThisHold = new AtomicBoolean(false);
        AtomicBoolean flywheelReadyLatchedThisHold = new AtomicBoolean(false);
        AtomicBoolean alignmentReadyLatchedThisHold = new AtomicBoolean(false);
        return Commands.run(() -> {
            Pose2d robotPose = poseSupplier.get();
            ChassisSpeeds robotSpeeds = speedsSupplier.get();
            var result = getAimResult(robotPose, robotSpeeds, useRobotSideForFeedTarget);
            shooter.setSetpoint(result.solution());

            boolean hoodAtSetpoint = shooter.isHoodAtSetpoint();
            boolean flywheelAtSetpoint = shooter.isFlywheelAtSetpoint();
            if (flywheelAtSetpoint) {
                flywheelReadyLatchedThisHold.set(true);
            }
            boolean flywheelReadyForRelease = shouldTreatFlywheelAsReady(
                    flywheelAtSetpoint,
                    releaseStartedThisHold.get(),
                    flywheelReadyLatchedThisHold.get());
            if (result.isAligned()) {
                alignmentReadyLatchedThisHold.set(true);
            }
            boolean alignedForRelease = shouldTreatAlignmentAsReady(
                    result.isAligned(),
                    alignmentReadyLatchedThisHold.get(),
                    result.rotationErrorRad());
            double alignmentToleranceRad = alignmentReadyLatchedThisHold.get()
                    ? kAimLatchedAlignmentToleranceRad
                    : kAimAlignmentToleranceRad;
            boolean shooterReadyForRelease = hoodAtSetpoint && flywheelReadyForRelease;
            boolean shotValid = result.solution().isValid();
            boolean sotmFireSafe = isSotmFireSafe(result.solution(), robotSpeeds);
            boolean canFire = alignedForRelease
                    && shotValid
                    && shooterReadyForRelease
                    && sotmFireSafe;
            boolean shotIsFeedMode = AutoAim.isFeedModeActive(robotPose);
            boolean topSensorBlocked = hopper.isTopBeamBreakBlocked();
            var timedFireDecision = TimedFirePolicy.evaluate(
                    true,
                    matchTimingService.timingKnown(),
                    matchTimingService.zoneActive(),
                    matchTimingService.zoneRemainingSec(),
                    armedDuringInactiveThisHold.get(),
                    topSensorBlocked,
                    shooterReadyForRelease,
                    alignedForRelease,
                    shotValid,
                    shotIsFeedMode,
                    result.solution());
            if (timedFireDecision.allowFeed()) {
                timedReleaseStartedThisHold.set(true);
            }
            Translation2d limitedTranslation = limitTranslationForLaunch(
                    robotPose,
                    result.solution(),
                    vxSupplier.get(),
                    vySupplier.get());

            boolean releaseAllowed = shouldAllowRelease(
                    canFire,
                    releaseStartedThisHold.get());
            String releaseBlockReason = determineReleaseBlockReason(
                    timedFireDecision,
                    timedReleaseStartedThisHold.get(),
                    shotValid,
                    hoodAtSetpoint,
                    flywheelReadyForRelease,
                    alignedForRelease,
                    sotmFireSafe);
            if (releaseAllowed) {
                releaseStartedThisHold.set(true);
            }
            // Continuous fuel should pause immediately when the live release gate drops.
            shooter.setFiring(releaseAllowed);
            hopper.setRunning(releaseAllowed);
            publishReleaseBlockTelemetry(!releaseAllowed, releaseBlockReason);
            publishReleaseConditionTelemetry(new ReleaseTelemetryInputs(
                    canFire,
                    releaseAllowed,
                    releaseStartedThisHold.get(),
                    shotValid,
                    hoodAtSetpoint,
                    flywheelAtSetpoint,
                    flywheelReadyForRelease,
                    alignedForRelease,
                    alignmentToleranceRad,
                    result.rotationErrorRad(),
                    sotmFireSafe,
                    robotSpeeds,
                    result.solution(),
                    timedFireDecision.timedAutoArmCandidate(),
                    armedDuringInactiveThisHold.get(),
                    timedReleaseStartedThisHold.get(),
                    matchTimingService.timingKnown(),
                    matchTimingService.zoneActive(),
                    topSensorBlocked,
                    timedFireDecision.releaseWindowOpen(),
                    matchTimingService.zoneRemainingSec(),
                    timedFireDecision.launchLeadSec(),
                    timedFireDecision.allowFeed(),
                    shotIsFeedMode));

            // ---- COR shifting + O-Lock (ported from MechAdv DriveCommands) ----
            // Convert operator-perspective translation to actual field-relative
            boolean allianceFlip = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;
            double fieldVx = allianceFlip ? -limitedTranslation.getX() : limitedTranslation.getX();
            double fieldVy = allianceFlip ? -limitedTranslation.getY() : limitedTranslation.getY();

            // Compute COR scalar — shifts rotation center toward shooter when yaw error is
            // large
            double yawErrorDeg = Math.toDegrees(Math.abs(result.rotationErrorRad()));
            double corScalar = MathUtil.clamp(
                    (yawErrorDeg - kCORMinErrorDeg) / (kCORMaxErrorDeg - kCORMinErrorDeg),
                    0.0, 1.0);

            // Shooter-to-robot offset (negative of robot-to-shooter offset)
            Translation2d shooterToRobot = new Translation2d(
                    -ShooterConstants.kShooterOffsetX.in(Meters), 0.0);

            // Apply COR shift via velocity transform
            ChassisSpeeds fieldSpeedsWithCOR = GeomUtil.transformVelocity(
                    new ChassisSpeeds(fieldVx, fieldVy, result.rotationCommand()),
                    shooterToRobot.times(1.0 - corScalar),
                    robotPose.getRotation());

            // O-Lock: X-lock wheels when near-stationary and aligned
            boolean oLock = Math.hypot(
                    fieldSpeedsWithCOR.vxMetersPerSecond,
                    fieldSpeedsWithCOR.vyMetersPerSecond) < kOLockSpeedThresholdMps
                    && Math.abs(fieldSpeedsWithCOR.omegaRadiansPerSecond) < kOLockOmegaThresholdRadPerSec;

            if (oLock) {
                swerve.stopWithXLock();
            } else {
                swerve.driveFieldRelative(fieldSpeedsWithCOR);
            }

            // Telemetry
            publishTelemetry(
                    true,
                    alignedForRelease,
                    shotValid,
                    sotmFireSafe,
                    canFire,
                    releaseAllowed,
                    result.rotationCommand(),
                    result.rotationErrorRad(),
                    limitedTranslation.getNorm());
            telemetry.publishTimedShotState(new TimedShotSnapshot(
                    timedFireDecision.timedAutoArmCandidate(),
                    armedDuringInactiveThisHold.get(),
                    timedReleaseStartedThisHold.get(),
                    matchTimingService.timingKnown(),
                    matchTimingService.zoneActive(),
                    topSensorBlocked,
                    timedFireDecision.releaseWindowOpen(),
                    matchTimingService.zoneRemainingSec(),
                    timedFireDecision.launchLeadSec(),
                    timedFireDecision.allowFeed(),
                    timedFireDecision.reasonCode(),
                    shotIsFeedMode),
                    TimedFirePolicy.reasonLabel(timedFireDecision.reasonCode()));

        }, this, swerve).finallyDo(() -> {
            shooter.stop();
            hopper.stop();
            publishReleaseBlockTelemetry(false, null);
            armedDuringInactiveThisHold.set(false);
            timedReleaseStartedThisHold.set(false);
            releaseStartedThisHold.set(false);
            flywheelReadyLatchedThisHold.set(false);
            alignmentReadyLatchedThisHold.set(false);
            resetAimSetpointDerivatives();
            telemetry.publishDriveAutoAim(false, 0.0, 0.0);
            resetTimedShotState();
            swerve.setFieldSpeeds(0.0, 0.0, 0.0);
        }).beforeStarting(() -> {
            armedDuringInactiveThisHold.set(matchTimingService.timingKnown() && !matchTimingService.zoneActive());
            timedReleaseStartedThisHold.set(false);
            releaseStartedThisHold.set(false);
            flywheelReadyLatchedThisHold.set(false);
            alignmentReadyLatchedThisHold.set(false);
            resetAimSetpointDerivatives();
            telemetry.publishDriveAutoAim(false, 0.0, 0.0);
            resetTimedShotState();
        })
                .withName("Superstructure AimAndShoot");
    }

    /**
     * Shoot without auto-aligning — uses AutoAim to set flywheel/hood but does
     * NOT control swerve rotation. The driver retains full manual drivetrain
     * control.
     */
    public Command shootNoAlignCommand() {
        AtomicBoolean releaseStartedThisHold = new AtomicBoolean(false);
        AtomicBoolean flywheelReadyLatchedThisHold = new AtomicBoolean(false);
        return Commands.run(() -> {
            var result = getAimResult();
            shooter.setSetpoint(result.solution());

            boolean hoodAtSetpoint = shooter.isHoodAtSetpoint();
            boolean flywheelAtSetpoint = shooter.isFlywheelAtSetpoint();
            if (flywheelAtSetpoint) {
                flywheelReadyLatchedThisHold.set(true);
            }
            boolean flywheelReadyForRelease = shouldTreatFlywheelAsReady(
                    flywheelAtSetpoint,
                    releaseStartedThisHold.get(),
                    flywheelReadyLatchedThisHold.get());
            boolean shotValid = result.solution().isValid();
            boolean sotmFireSafe = isSotmFireSafe(result.solution(), speedsSupplier.get());
            boolean canFire = shotValid && hoodAtSetpoint && flywheelReadyForRelease;
            String releaseBlockReason = determineLiveReleaseBlockReason(
                    shotValid,
                    hoodAtSetpoint,
                    flywheelReadyForRelease,
                    true,
                    sotmFireSafe);
            if (canFire) {
                releaseStartedThisHold.set(true);
            }
            boolean releaseAllowed = canFire || releaseStartedThisHold.get();

            shooter.setFiring(releaseAllowed);
            hopper.setRunning(releaseAllowed);
            publishReleaseBlockTelemetry(!canFire, releaseBlockReason);
            publishReleaseConditionTelemetry(new ReleaseTelemetryInputs(
                    canFire,
                    canFire,
                    releaseStartedThisHold.get(),
                    shotValid,
                    hoodAtSetpoint,
                    flywheelAtSetpoint,
                    flywheelReadyForRelease,
                    true,
                    kAimAlignmentToleranceRad,
                    result.rotationErrorRad(),
                    sotmFireSafe,
                    speedsSupplier.get(),
                    result.solution(),
                    false,
                    false,
                    false,
                    false,
                    false,
                    hopper.isTopBeamBreakBlocked(),
                    false,
                    0.0,
                    0.0,
                    false,
                    AutoAim.isFeedModeActive(poseSupplier.get())));

            publishTelemetry(
                    false,
                    result.isAligned(),
                    shotValid,
                    sotmFireSafe,
                    canFire,
                    canFire,
                    0.0,
                    result.rotationErrorRad(),
                    0.0);
        }, this).beforeStarting(() -> {
        }).finallyDo(() -> {
            shooter.stop();
            hopper.stop();
            publishReleaseBlockTelemetry(false, null);
            releaseStartedThisHold.set(false);
            flywheelReadyLatchedThisHold.set(false);
        })
                .withName("Superstructure ShootNoAlign");
    }

    /**
     * Idle command - stops all mechanisms.
     */
    public Command idleCommand() {
        return Commands.runOnce(() -> {
            shooter.stop();
            hopper.stop();
        }, this).withName("Superstructure Idle");
    }

    // ==================== ACCESSORS ====================

    public Shooter getShooter() {
        return shooter;
    }

    public Intake getIntake() {
        return intake;
    }

    public Hopper getHopper() {
        return hopper;
    }

    private void publishPreloadState(boolean active, int phaseCode, int faultCode) {
        telemetry.publishHopperPreloadState(new HopperPreloadSnapshot(active, phaseCode, faultCode));
    }

    private void resetTimedShotState() {
        telemetry.publishTimedShotState(new TimedShotSnapshot(
                false,
                false,
                false,
                false,
                false,
                false,
                false,
                0.0,
                0.0,
                false,
                TimedFirePolicy.REASON_NOT_REQUESTED,
                false),
                TimedFirePolicy.reasonLabel(TimedFirePolicy.REASON_NOT_REQUESTED));
    }

    private void publishTelemetry(
            boolean autoAimActive,
            boolean isAligned,
            boolean shotValid,
            boolean sotmFireSafe,
            boolean canFire,
            boolean releaseAllowed,
            double rotCmd,
            double rotErrorRad,
            double transLimitMps) {
        telemetry.publishDriveAutoAim(autoAimActive, rotCmd, transLimitMps);
        telemetry.publishSuperstructureState(new SuperstructureSnapshot(
                isAligned,
                shotValid,
                sotmFireSafe,
                canFire,
                releaseAllowed,
                rotCmd,
                rotErrorRad));
    }
}
