package com.team9470.subsystems;

import com.team9470.TunerConstants;
import com.team9470.subsystems.hopper.Hopper;
import com.team9470.subsystems.intake.Intake;
import com.team9470.subsystems.shooter.Shooter;
import com.team9470.telemetry.TelemetryManager;
import com.team9470.telemetry.structs.SuperstructureSnapshot;
import com.team9470.util.AutoAim;

import com.team9470.subsystems.swerve.Swerve;

import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
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
                    intake.setShooting(true);
                    intake.setAgitating(true);
                },
                () -> {
                    shooter.stop();
                    hopper.stop();
                    intake.setShooting(false);
                    intake.setAgitating(false);
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

    /**
     * Aim, rotate, and shoot — auto version (robot stationary, only rotates to
     * aim).
     */
    public Command aimAndShootCommand() {
        return aimAndShootCommand(() -> 0.0, () -> 0.0, true);
    }

    /**
     * Aim, rotate, and shoot — teleop version (driver controls translation).
     * Handles swerve rotation automatically via auto-aim.
     *
     * @param vxSupplier field-relative X velocity (m/s)
     * @param vySupplier field-relative Y velocity (m/s)
     */
    public Command aimAndShootCommand(Supplier<Double> vxSupplier, Supplier<Double> vySupplier) {
        return aimAndShootCommand(vxSupplier, vySupplier, true);
    }

    /**
     * Aim, rotate, and shoot — full version.
     * Handles swerve rotation automatically via auto-aim.
     *
     * @param vxSupplier field-relative X velocity (m/s)
     * @param vySupplier field-relative Y velocity (m/s)
     * @param agitate    whether to agitate the intake during shooting
     */
    public Command aimAndShootCommand(Supplier<Double> vxSupplier, Supplier<Double> vySupplier, boolean agitate) {
        return aimAndShootCommand(vxSupplier, vySupplier, agitate, !agitate);
    }

    /**
     * Aim, rotate, and shoot — full version with explicit feed-side targeting
     * selection.
     * Handles swerve rotation automatically via auto-aim.
     *
     * @param vxSupplier                   field-relative X velocity (m/s)
     * @param vySupplier                   field-relative Y velocity (m/s)
     * @param agitate                      whether to agitate the intake during
     *                                     shooting
     * @param useRobotSideForFeedTarget    when true, feed mode target side is
     *                                     selected from robot side (left/right)
     */
    public Command aimAndShootCommand(
            Supplier<Double> vxSupplier,
            Supplier<Double> vySupplier,
            boolean agitate,
            boolean useRobotSideForFeedTarget) {
        Swerve swerve = Swerve.getInstance();
        AtomicBoolean shooterReadyLatched = new AtomicBoolean(false);
        // Use closed-loop velocity so commanding 0 m/s actively brakes (no drift)
        SwerveRequest.FieldCentric aimDrive = new SwerveRequest.FieldCentric()
                .withDriveRequestType(SwerveModule.DriveRequestType.Velocity);
        return Commands.run(() -> {
            Pose2d robotPose = poseSupplier.get();
            ChassisSpeeds robotSpeeds = speedsSupplier.get();
            var result = getAimResult(robotPose, robotSpeeds, useRobotSideForFeedTarget);
            shooter.setSetpoint(result.solution());
            intake.setShooting(true);
            intake.setAgitating(agitate);

            boolean shooterAtSetpoint = shooter.isAtSetpoint();
            if (shooterAtSetpoint) {
                shooterReadyLatched.set(true);
            }

            boolean canFire = result.isAligned()
                    && result.solution().isValid()
                    && shooterAtSetpoint
                    && isSotmFireSafe(result.solution(), robotSpeeds);
            Translation2d limitedTranslation = limitTranslationForLaunch(
                    robotPose,
                    result.solution(),
                    vxSupplier.get(),
                    vySupplier.get());

            // Wait for initial shooter readiness, then feed continuously.
            shooter.setFiring(canFire);
            hopper.setRunning(shooterReadyLatched.get());

            // Drive: pass through translation, auto-aim rotation (closed-loop velocity)
            swerve.setControl(aimDrive
                    .withVelocityX(limitedTranslation.getX())
                    .withVelocityY(limitedTranslation.getY())
                    .withRotationalRate(result.rotationCommand()));

            // Telemetry
            publishTelemetry(result.isAligned(), canFire, result.rotationCommand(), result.rotationErrorRad());

        }, this, swerve).finallyDo(() -> {
            shooter.stop();
            hopper.stop();
            intake.setShooting(false);
            intake.setAgitating(false);
            shooterReadyLatched.set(false);
            resetAimSetpointDerivatives();
            swerve.setControl(aimDrive.withVelocityX(0).withVelocityY(0).withRotationalRate(0));
        }).beforeStarting(() -> {
            shooterReadyLatched.set(false);
            resetAimSetpointDerivatives();
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
            intake.setShooting(true);
            intake.setAgitating(true);

            boolean shooterAtSetpoint = shooter.isAtSetpoint();
            if (shooterAtSetpoint) {
                shooterReadyLatched.set(true);
            }

            boolean canFire = result.solution().isValid() && shooterAtSetpoint;

            shooter.setFiring(canFire);
            hopper.setRunning(shooterReadyLatched.get());

            publishTelemetry(false, canFire, 0.0, 0.0);
        }, this).finallyDo(() -> {
            shooter.stop();
            hopper.stop();
            intake.setShooting(false);
            intake.setAgitating(false);
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
            intake.setShooting(false);
            intake.setAgitating(false);
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

    private void publishTelemetry(boolean isAligned, boolean canFire, double rotCmd, double rotErrorRad) {
        telemetry.publishSuperstructureState(new SuperstructureSnapshot(isAligned, canFire, rotCmd, rotErrorRad));
    }
}
