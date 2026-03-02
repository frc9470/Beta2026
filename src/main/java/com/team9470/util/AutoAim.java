package com.team9470.util;

import com.team9470.FieldConstants;
import com.team9470.telemetry.TelemetryManager;
import com.team9470.telemetry.structs.AutoAimSolverSnapshot;
import com.team9470.subsystems.shooter.ShooterConstants;
import com.team9470.subsystems.shooter.ShooterInterpolationMaps;
import com.team9470.subsystems.shooter.ShotParameter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import static edu.wpi.first.units.Units.*;

import java.util.Optional;

public class AutoAim {
    private static final TelemetryManager telemetry = TelemetryManager.getInstance();

    // Field Geometry (from FieldConstants)
    private static final double BALL_RADIUS_METERS = FieldConstants.GamePiece.ballRadius;
    private static final double GOAL_Z = FieldConstants.Hub.height + BALL_RADIUS_METERS; // Target Z
    private static final double FEED_MODE_BLUE_X_THRESHOLD_M = 4.5;

    // Field Center Target (Hub center point + ball clearance)
    private static final Translation3d BASE_HUB_TARGET = new Translation3d(
            FieldConstants.Hub.topCenterPoint.getX(),
            FieldConstants.Hub.topCenterPoint.getY(),
            GOAL_Z);
    private static final Translation3d BASE_FEED_LEFT_TARGET = new Translation3d(1.8, 6.7, GOAL_Z);
    private static final Translation3d BASE_FEED_RIGHT_TARGET = new Translation3d(
            BASE_FEED_LEFT_TARGET.getX(),
            FieldConstants.fieldWidth - BASE_FEED_LEFT_TARGET.getY(),
            GOAL_Z);

    // Shooter Configuration
    // Exit point relative to robot center
    public static final Translation3d SHOOTER_OFFSET = new Translation3d(
            ShooterConstants.kShooterOffsetX,
            Meters.of(0.0),
            ShooterConstants.kShooterOffsetZ);

    private enum AimMode {
        HUB,
        FEED
    }

    // --- Result Record ---
    public record ShootingSolution(
            Rotation2d targetRobotYaw,
            double hoodCommandDeg,
            double flywheelRpm,
            double targetOmega,
            double distanceMeters,
            double distanceNoLookaheadMeters,
            double naiveAirTimeSec,
            double shooterFieldSpeedMps,
            double shooterFieldAccelMps2,
            boolean isValid) {
    }

    /**
     * Returns the target position (Field-Relative), flipped based on alliance.
     */
    public static Translation3d getTarget() {
        return AllianceFlipUtil.apply(BASE_HUB_TARGET);
    }

    /**
     * Returns the target position (Field-Relative), flipped based on alliance and
     * selected auto mode.
     */
    public static Translation3d getTarget(Pose2d robotPose) {
        return getTarget(robotPose, false);
    }

    /**
     * Returns the target position (Field-Relative), flipped based on alliance.
     * In feed mode, optional side selection can choose left/right feed target from
     * the robot Y position in the blue reference frame.
     */
    public static Translation3d getTarget(Pose2d robotPose, boolean useRobotSideForFeedTarget) {
        if (robotPose == null) {
            return getTarget();
        }
        AimMode mode = getAimMode(robotPose);
        Translation3d baseTarget = (mode == AimMode.FEED)
                ? getFeedTarget(robotPose, useRobotSideForFeedTarget)
                : BASE_HUB_TARGET;
        return AllianceFlipUtil.apply(baseTarget);
    }

    /**
     * Returns true when feed mode is active for the provided pose.
     */
    public static boolean isFeedModeActive(Pose2d robotPose) {
        if (robotPose == null) {
            return false;
        }
        return getAimMode(robotPose) == AimMode.FEED;
    }

    /**
     * Returns the robot X position converted to the blue reference frame.
     */
    public static double getRobotXBlueMeters(Pose2d robotPose) {
        if (robotPose == null) {
            return 0.0;
        }
        return AllianceFlipUtil.applyX(robotPose.getX());
    }

    /**
     * Publishes mode telemetry independent of active shooting state.
     */
    public static void publishModeTelemetry(Pose2d robotPose) {
        double robotXBlueMeters = getRobotXBlueMeters(robotPose);
        AimMode mode = isFeedModeActive(robotPose) ? AimMode.FEED : AimMode.HUB;
        telemetry.publishAutoAimSolver(new AutoAimSolverSnapshot(
                mode == AimMode.FEED ? 1 : 0,
                mode == AimMode.FEED,
                robotXBlueMeters,
                Double.NaN,
                false,
                Double.NaN,
                Double.NaN));
    }

    // Number of iterations for the SOTM lookahead convergence loop.
    private static final int LOOKAHEAD_ITERATIONS = 20;
    // Pose lead to reduce control/actuation latency error.
    private static final double CONTROL_PHASE_DELAY_SEC = 0.03;
    // Blend out SOTM compensation near standstill to protect stationary shots.
    private static final double SOTM_BLEND_START_MPS = 0.15;
    private static final double SOTM_BLEND_FULL_MPS = 0.60;
    // Higher-risk moving-shot terms (acceleration + rotational exit velocity).
    private static final double NOMINAL_LOOP_DT_SEC = 0.02;
    private static final double MAX_ACCELERATION_MPS2 = 6.0;
    private static final double MAX_ACCEL_LOOKAHEAD_SEC = 0.35;
    private static final double ACCEL_FILTER_ALPHA = 0.25;
    private static Translation2d lastShooterFieldVelocity = new Translation2d();
    private static Translation2d filteredShooterFieldAcceleration = new Translation2d();
    private static boolean motionHistoryInitialized = false;

    /**
     * Calculates the shooting solution with Shoot-on-the-Move (SOTM) support.
     *
     * <p>
     * When the robot is moving, the iterative lookahead shifts the aiming
     * target by -(fieldVelocity × airTime) so the projectile lands at the
     * real target position even though the robot (and therefore the shooter
     * exit point) has moved during the time-of-flight.
     */
    public static ShootingSolution calculate(Pose2d robotPose, ChassisSpeeds robotSpeeds) {
        return calculate(robotPose, robotSpeeds, false);
    }

    /**
     * Calculates the shooting solution with Shoot-on-the-Move (SOTM) support.
     *
     * @param useRobotSideForFeedTarget when true, feed mode target side is selected
     *                                  from robot Y position (left side aims left
     *                                  target, right side aims right target)
     */
    public static ShootingSolution calculate(
            Pose2d robotPose,
            ChassisSpeeds robotSpeeds,
            boolean useRobotSideForFeedTarget) {
        if (robotPose == null || robotSpeeds == null) {
            publishMapTelemetry(0.0, null, AimMode.HUB, false, 0.0);
            return new ShootingSolution(new Rotation2d(), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, false);
        }

        // Predict near-future pose to account for control/actuation latency.
        Pose2d predictedPose = robotPose.exp(new Twist2d(
                robotSpeeds.vxMetersPerSecond * CONTROL_PHASE_DELAY_SEC,
                robotSpeeds.vyMetersPerSecond * CONTROL_PHASE_DELAY_SEC,
                robotSpeeds.omegaRadiansPerSecond * CONTROL_PHASE_DELAY_SEC));

        double robotXBlueMeters = getRobotXBlueMeters(predictedPose);
        AimMode mode = getAimMode(predictedPose);
        boolean feedModeActive = mode == AimMode.FEED;
        Translation3d baseTarget3d = getTarget(predictedPose, useRobotSideForFeedTarget);
        Translation2d baseTargetXY = baseTarget3d.toTranslation2d();

        // Shooter exit point on the field (accounts for robot rotation).
        Translation2d shooterOffsetXY = new Translation2d(SHOOTER_OFFSET.getX(), SHOOTER_OFFSET.getY())
                .rotateBy(predictedPose.getRotation());
        Translation2d shooterExitXY = predictedPose.getTranslation().plus(shooterOffsetXY);

        // Convert chassis speeds to field-relative velocity (Translation2d).
        ChassisSpeeds fieldSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(
                robotSpeeds, predictedPose.getRotation());
        Translation2d fieldLinearVelocity = new Translation2d(
                fieldSpeeds.vxMetersPerSecond,
                fieldSpeeds.vyMetersPerSecond);
        double fieldSpeedMps = fieldLinearVelocity.getNorm();
        double sotmBlend = MathUtil.clamp(
                (fieldSpeedMps - SOTM_BLEND_START_MPS) / (SOTM_BLEND_FULL_MPS - SOTM_BLEND_START_MPS),
                0.0,
                1.0);
        // Include tangential velocity from robot yaw at the shooter exit point.
        Translation2d rotationalVelocityAtShooter = new Translation2d(
                -robotSpeeds.omegaRadiansPerSecond * shooterOffsetXY.getY(),
                robotSpeeds.omegaRadiansPerSecond * shooterOffsetXY.getX());
        Translation2d shooterFieldVelocity = fieldLinearVelocity.plus(rotationalVelocityAtShooter.times(sotmBlend));

        Translation2d shooterFieldAcceleration = new Translation2d();
        if (motionHistoryInitialized) {
            Translation2d rawAcceleration = shooterFieldVelocity.minus(lastShooterFieldVelocity).div(NOMINAL_LOOP_DT_SEC);
            rawAcceleration = clampVectorNorm(rawAcceleration, MAX_ACCELERATION_MPS2);
            filteredShooterFieldAcceleration = filteredShooterFieldAcceleration.times(1.0 - ACCEL_FILTER_ALPHA)
                    .plus(rawAcceleration.times(ACCEL_FILTER_ALPHA));
            shooterFieldAcceleration = filteredShooterFieldAcceleration;
        } else {
            motionHistoryInitialized = true;
            filteredShooterFieldAcceleration = new Translation2d();
            shooterFieldAcceleration = filteredShooterFieldAcceleration;
        }
        lastShooterFieldVelocity = shooterFieldVelocity;
        double shooterFieldSpeedMps = shooterFieldVelocity.getNorm();
        double shooterFieldAccelMps2 = shooterFieldAcceleration.getNorm();

        double distanceNoLookaheadMeters = shooterExitXY.getDistance(baseTargetXY);
        double naiveAirTimeSec = ShooterInterpolationMaps.getAirTime(distanceNoLookaheadMeters);
        if (!Double.isFinite(naiveAirTimeSec) || naiveAirTimeSec < 0.0) {
            naiveAirTimeSec = 0.0;
        }

        // ---- Iterative lookahead (ported from Ninja RobotState) ----
        // Each iteration: compute distance → look up air time → shift the
        // virtual target by -(velocity × airTime). After convergence the
        // shifted target already accounts for where the robot will be when
        // the projectile arrives.
        Translation2d lookaheadTarget = baseTargetXY;
        for (int i = 0; i < LOOKAHEAD_ITERATIONS; i++) {
            double dist = shooterExitXY.getDistance(lookaheadTarget);
            double airTime = ShooterInterpolationMaps.getAirTime(dist);
            if (!Double.isFinite(airTime) || airTime < 0.0) {
                airTime = 0.0;
            }
            Translation2d offset = shooterFieldVelocity.times(airTime * sotmBlend);
            if (sotmBlend > 0.0) {
                double accelTime = Math.min(airTime, MAX_ACCEL_LOOKAHEAD_SEC);
                offset = offset.plus(shooterFieldAcceleration.times(0.5 * accelTime * accelTime * sotmBlend));
            }
            lookaheadTarget = baseTargetXY.minus(offset);
        }

        // Distance and shot parameter lookup use the converged lookahead target.
        double distanceMeters = shooterExitXY.getDistance(lookaheadTarget);
        Optional<ShotParameter> shotParameter = mode == AimMode.FEED
                ? ShooterInterpolationMaps.getFeed(distanceMeters)
                : ShooterInterpolationMaps.getHub(distanceMeters);
        publishMapTelemetry(distanceMeters, shotParameter.orElse(null), mode, feedModeActive, robotXBlueMeters);

        // Yaw to converged lookahead target.
        double dx = lookaheadTarget.getX() - predictedPose.getX();
        double dy = lookaheadTarget.getY() - predictedPose.getY();
        Rotation2d targetRobotYaw = new Rotation2d(dx, dy);

        if (shotParameter.isEmpty() || !shotParameter.get().isValid()) {
            return new ShootingSolution(
                    targetRobotYaw,
                    0.0,
                    0.0,
                    0.0,
                    distanceMeters,
                    distanceNoLookaheadMeters,
                    naiveAirTimeSec,
                    shooterFieldSpeedMps,
                    shooterFieldAccelMps2,
                    false);
        }

        // ---- Angular feedforward (targetOmega) ----
        // Approximate dθ/dt of the lookahead angle due to robot velocity so
        // the swerve rotation controller can track the moving setpoint.
        // For a static target, dθ/dt = (vx*dy - vy*dx) / r^2.
        double yawDistanceMeters = Math.max(predictedPose.getTranslation().getDistance(lookaheadTarget), 0.5);
        double losAngle = targetRobotYaw.getRadians();
        double targetOmega = (shooterFieldVelocity.getX() * Math.sin(losAngle)
                - shooterFieldVelocity.getY() * Math.cos(losAngle)) / yawDistanceMeters;
        targetOmega *= sotmBlend;

        ShotParameter shot = shotParameter.get();
        return new ShootingSolution(
                targetRobotYaw,
                shot.hoodCommandDeg(),
                shot.flywheelRpm(),
                targetOmega,
                distanceMeters,
                distanceNoLookaheadMeters,
                naiveAirTimeSec,
                shooterFieldSpeedMps,
                shooterFieldAccelMps2,
                true);
    }

    private static Translation2d clampVectorNorm(Translation2d vector, double maxNorm) {
        double norm = vector.getNorm();
        if (norm <= maxNorm || norm <= 1e-9) {
            return vector;
        }
        return vector.times(maxNorm / norm);
    }

    private static AimMode getAimMode(Pose2d robotPose) {
        double robotXBlueMeters = AllianceFlipUtil.applyX(robotPose.getX());
        return robotXBlueMeters > FEED_MODE_BLUE_X_THRESHOLD_M ? AimMode.FEED : AimMode.HUB;
    }

    private static Translation3d getFeedTarget(Pose2d robotPose, boolean useRobotSideForFeedTarget) {
        if (!useRobotSideForFeedTarget || robotPose == null) {
            return BASE_FEED_LEFT_TARGET;
        }
        double robotYBlueMeters = AllianceFlipUtil.applyY(robotPose.getY());
        return robotYBlueMeters >= FieldConstants.LinesHorizontal.center
                ? BASE_FEED_LEFT_TARGET
                : BASE_FEED_RIGHT_TARGET;
    }

    private static void publishMapTelemetry(
            double distanceMeters,
            ShotParameter shot,
            AimMode mode,
            boolean feedModeActive,
            double robotXBlueMeters) {
        boolean valid = shot != null && shot.isValid();
        telemetry.publishAutoAimSolver(new AutoAimSolverSnapshot(
                mode == AimMode.FEED ? 1 : 0,
                feedModeActive,
                robotXBlueMeters,
                distanceMeters,
                valid,
                valid ? Math.toRadians(shot.hoodCommandDeg()) : 0.0,
                valid ? shot.flywheelRpm() / 60.0 : 0.0));
    }
}
