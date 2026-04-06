package com.team9470.subsystems;

import com.team9470.TunerConstants;
import com.team9470.subsystems.hopper.Hopper;
import com.team9470.subsystems.intake.Intake;
import com.team9470.subsystems.shooter.Shooter;
import com.team9470.telemetry.MatchTimingService;
import com.team9470.telemetry.TelemetryManager;
import com.team9470.telemetry.structs.HopperPreloadSnapshot;
import com.team9470.telemetry.structs.SuperstructureSnapshot;
import com.team9470.telemetry.structs.TimedShotSnapshot;
import com.team9470.util.AutoAim;
import com.team9470.util.TimedFirePolicy;

import com.team9470.subsystems.swerve.Swerve;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

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
    private static final double kAimKp = 12.0;
    private static final double kAimKd = 0.5;
    private static final double kAimAlignmentToleranceRad = Math.toRadians(3.0);
    private static final double kAimMaxAngularRateRadPerSec = Math.toRadians(TunerConstants.maxAngularVelocity);
    private static final double kDriveAngleDerivativeFilterWindowSec = 1.5;
    private static final double kHoodDerivativeFilterWindowSec = 0.4;
    private static final double kFlywheelDerivativeFilterWindowSec = 0.4;
    private static final double kShootMaxPolarVelocityRadPerSec = 0.5;
    private static final double kShootVelocityLimitMinRequestMps = 0.15;
    private static final double kSotmFireSafetyMinSpeedMps = 0.35;
    private static final double kSotmMaxAccelerationForFireMps2 = 3.5;
    private static final double kSotmMaxOmegaForFireRadPerSec = Math.toRadians(220.0);
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
    private LinearFilter driveAngleRateFilter = createMovingAverageFilter(kDriveAngleDerivativeFilterWindowSec);
    private LinearFilter hoodRateFilter = createMovingAverageFilter(kHoodDerivativeFilterWindowSec);
    private LinearFilter flywheelRateFilter = createMovingAverageFilter(kFlywheelDerivativeFilterWindowSec);
    private boolean derivativeStateInitialized = false;
    private Rotation2d lastTargetYaw = new Rotation2d();
    private double lastHoodCommandDeg = 0.0;
    private double lastFlywheelRpm = 0.0;

    private Superstructure() {
        shooter = new Shooter();
        intake = Intake.getInstance();
        hopper = Hopper.getInstance();
        publishPreloadState(false, PRELOAD_PHASE_IDLE, PRELOAD_FAULT_NONE);
        resetTimedShotState();
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
        // No unconditional setSetpoint here - AutoAim setpoints are applied
        // only by shooting commands (shootCommand, aimAndShootCommand).
        // This prevents overriding manual flywheel/hood commands (e.g. debug Y button).
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
        final int[] phaseCode = { PRELOAD_PHASE_IDLE };
        final int[] faultCode = { PRELOAD_FAULT_NONE };
        final boolean[] finished = { false };
        final double[] commandStartSec = { Double.NaN };
        final double[] phaseStartSec = { Double.NaN };
        final double[] jamStartSec = { Double.NaN };

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

                double averageFeederCurrentAmps = (
                        Math.abs(hopper.getFeederLeftStatorCurrentAmps()) + Math.abs(hopper.getFeederRightStatorCurrentAmps()))
                        * 0.5;
                double averageFeederVelocityRps = (
                        Math.abs(hopper.getFeederLeftVelocityRps()) + Math.abs(hopper.getFeederRightVelocityRps()))
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
        }, this, hopper).beforeStarting(() -> {
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

        double rawDriveAngleRateRadPerSec = solution.targetRobotYaw().minus(lastTargetYaw).getRadians() / kControlLoopDtSec;
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
            boolean timedAutoArmCandidate,
            boolean timedReleaseStartedThisHold,
            boolean canFire,
            boolean timedAllowFeed) {
        if (!timedAutoArmCandidate || timedReleaseStartedThisHold) {
            return canFire;
        }
        return timedAllowFeed;
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
     * @param vxSupplier                   field-relative X velocity (m/s)
     * @param vySupplier                   field-relative Y velocity (m/s)
     * @param useRobotSideForFeedTarget    when true, feed mode target side is
     *                                     selected from robot side (left/right)
     */
    public Command aimAndShootCommand(
            Supplier<Double> vxSupplier,
            Supplier<Double> vySupplier,
            boolean useRobotSideForFeedTarget) {
        Swerve swerve = Swerve.getInstance();
        AtomicBoolean shooterReadyLatched = new AtomicBoolean(false);
        AtomicBoolean armedDuringInactiveThisHold = new AtomicBoolean(false);
        AtomicBoolean timedReleaseStartedThisHold = new AtomicBoolean(false);
        return Commands.run(() -> {
            Pose2d robotPose = poseSupplier.get();
            ChassisSpeeds robotSpeeds = speedsSupplier.get();
            var result = getAimResult(robotPose, robotSpeeds, useRobotSideForFeedTarget);
            shooter.setSetpoint(result.solution());

            boolean shooterAtSetpoint = shooter.isAtSetpoint();
            if (shooterAtSetpoint) {
                shooterReadyLatched.set(true);
            }
            boolean canFire = result.isAligned()
                    && result.solution().isValid()
                    && shooterAtSetpoint
                    && isSotmFireSafe(result.solution(), robotSpeeds);
            boolean shotIsFeedMode = AutoAim.isFeedModeActive(robotPose);
            var timedFireDecision = TimedFirePolicy.evaluate(
                    true,
                    matchTimingService.timingKnown(),
                    matchTimingService.zoneActive(),
                    matchTimingService.zoneRemainingSec(),
                    armedDuringInactiveThisHold.get(),
                    hopper.isTopBeamBreakBlocked(),
                    shooterAtSetpoint,
                    result.isAligned(),
                    result.solution().isValid(),
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
                    timedFireDecision.timedAutoArmCandidate(),
                    timedReleaseStartedThisHold.get(),
                    canFire,
                    timedFireDecision.allowFeed());

            boolean useNormalFeedPath = !timedFireDecision.timedAutoArmCandidate()
                    || timedReleaseStartedThisHold.get();
            boolean feederShouldRun = useNormalFeedPath
                    ? shooterReadyLatched.get()
                    : timedFireDecision.allowFeed();
            boolean shooterShouldFire = useNormalFeedPath
                    ? canFire
                    : timedFireDecision.allowFeed();

            shooter.setFiring(shooterShouldFire);
            hopper.setRunning(feederShouldRun);

            // Drive field-relative through FieldCentric (handles operator perspective).
            swerve.setFieldSpeeds(
                    limitedTranslation.getX(),
                    limitedTranslation.getY(),
                    result.rotationCommand());

            // Telemetry
            publishTelemetry(
                    true,
                    result.isAligned(),
                    shooterShouldFire,
                    result.rotationCommand(),
                    result.rotationErrorRad(),
                    limitedTranslation.getNorm());
            telemetry.publishTimedShotState(new TimedShotSnapshot(
                    timedFireDecision.timedAutoArmCandidate(),
                    armedDuringInactiveThisHold.get(),
                    timedReleaseStartedThisHold.get(),
                    matchTimingService.timingKnown(),
                    matchTimingService.zoneActive(),
                    matchTimingService.zoneRemainingSec(),
                    timedFireDecision.launchLeadSec(),
                    timedFireDecision.allowFeed(),
                    timedFireDecision.reasonCode(),
                    shotIsFeedMode));

        }, this, swerve).finallyDo(() -> {
            shooter.stop();
            hopper.stop();
            shooterReadyLatched.set(false);
            armedDuringInactiveThisHold.set(false);
            timedReleaseStartedThisHold.set(false);
            resetAimSetpointDerivatives();
            telemetry.publishDriveAutoAim(false, 0.0, 0.0);
            resetTimedShotState();
            swerve.setFieldSpeeds(0.0, 0.0, 0.0);
        }).beforeStarting(() -> {
            shooterReadyLatched.set(false);
            armedDuringInactiveThisHold.set(matchTimingService.timingKnown() && !matchTimingService.zoneActive());
            timedReleaseStartedThisHold.set(false);
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
        AtomicBoolean shooterReadyLatched = new AtomicBoolean(false);
        return Commands.run(() -> {
            var result = getAimResult();
            shooter.setSetpoint(result.solution());

            boolean shooterAtSetpoint = shooter.isAtSetpoint();
            if (shooterAtSetpoint) {
                shooterReadyLatched.set(true);
            }
            boolean canFire = result.solution().isValid() && shooterAtSetpoint;

            shooter.setFiring(canFire);
            hopper.setRunning(shooterReadyLatched.get());

            publishTelemetry(false, false, canFire, 0.0, 0.0, 0.0);
        }, this).finallyDo(() -> {
            shooter.stop();
            hopper.stop();
            shooterReadyLatched.set(false);
        }).beforeStarting(() -> shooterReadyLatched.set(false))
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
                0.0,
                0.0,
                false,
                TimedFirePolicy.REASON_NOT_REQUESTED,
                false));
    }

    private void publishTelemetry(
            boolean autoAimActive,
            boolean isAligned,
            boolean canFire,
            double rotCmd,
            double rotErrorRad,
            double transLimitMps) {
        telemetry.publishDriveAutoAim(autoAimActive, rotCmd, transLimitMps);
        telemetry.publishSuperstructureState(new SuperstructureSnapshot(isAligned, canFire, rotCmd, rotErrorRad));
    }
}
