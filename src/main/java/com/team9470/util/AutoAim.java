package com.team9470.util;

import static edu.wpi.first.units.Units.Meters;

import com.team9470.FieldConstants;
import com.team9470.subsystems.shooter.ShooterConstants;
import com.team9470.subsystems.shooter.ShooterInterpolationMaps;
import com.team9470.subsystems.shooter.ShotParameter;
import com.team9470.telemetry.TelemetryManager;
import com.team9470.telemetry.structs.AutoAimSolverSnapshot;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import java.util.Optional;

public class AutoAim {
    private static final TelemetryManager telemetry = TelemetryManager.getInstance();
    private static final boolean kEnableSotm = true;

    private static final double BALL_RADIUS_METERS = FieldConstants.GamePiece.ballRadius;
    private static final double GOAL_Z = FieldConstants.Hub.height + BALL_RADIUS_METERS;
    private static final double FEED_MODE_BLUE_X_THRESHOLD_M = 4.5;
    private static final double BASE_FEED_TARGET_X_M = 1.8;
    private static final double FEED_TARGET_Y_OFFSET_FROM_CENTER_M = 1.9952;
    private static final double OVER_BUMP_FEED_TARGET_X_M = 4.6;
    private static final double OVER_BUMP_FEED_TARGET_Y_M = 5.5;

    private static final Translation3d BASE_HUB_TARGET = new Translation3d(
            FieldConstants.Hub.topCenterPoint.getX(),
            FieldConstants.Hub.topCenterPoint.getY(),
            GOAL_Z);
    private static final Translation3d BASE_FEED_LEFT_TARGET = new Translation3d(
            BASE_FEED_TARGET_X_M,
            FieldConstants.LinesHorizontal.center + FEED_TARGET_Y_OFFSET_FROM_CENTER_M,
            GOAL_Z);
    private static final Translation3d BASE_FEED_RIGHT_TARGET = new Translation3d(
            BASE_FEED_LEFT_TARGET.getX(),
            FieldConstants.fieldWidth - BASE_FEED_LEFT_TARGET.getY(),
            GOAL_Z);
    private static final Translation3d BASE_OVER_BUMP_FEED_TARGET = new Translation3d(
            OVER_BUMP_FEED_TARGET_X_M,
            OVER_BUMP_FEED_TARGET_Y_M,
            GOAL_Z);

    public static final Translation3d SHOOTER_OFFSET = new Translation3d(
            ShooterConstants.kShooterOffsetX,
            Meters.of(0.0),
            ShooterConstants.kShooterOffsetZ);

    private enum AimMode {
        HUB,
        FEED
    }

    public enum FeedTargetMode {
        DEFAULT_LEFT,
        ROBOT_SIDE,
        OVER_BUMP
    }

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

    private record SolverRequest(
            Pose2d robotPose,
            AimMode mode,
            double robotXBlueMeters,
            Translation3d fieldTarget,
            Translation2d shooterExitXY) {
    }

    private record SolverState(
            AimMode mode,
            double robotXBlueMeters,
            Translation3d fieldTarget,
            Translation2d aimPoint,
            ShootingSolution solution) {
    }

    private record SotmMotionState(
            Pose2d predictedPose,
            Translation2d shooterFieldVelocity,
            Translation2d shooterFieldAcceleration,
            double shooterFieldSpeedMps,
            double shooterFieldAccelMps2,
            double sotmBlend) {
    }

    public static Translation3d getTarget() {
        return AllianceFlipUtil.apply(BASE_HUB_TARGET);
    }

    public static Translation3d getTarget(Pose2d robotPose) {
        return getTarget(robotPose, false);
    }

    public static Translation3d getTarget(Pose2d robotPose, boolean useRobotSideForFeedTarget) {
        return getTarget(robotPose, useRobotSideForFeedTarget ? FeedTargetMode.ROBOT_SIDE : FeedTargetMode.DEFAULT_LEFT);
    }

    public static Translation3d getTarget(Pose2d robotPose, FeedTargetMode feedTargetMode) {
        if (robotPose == null) {
            return getTarget();
        }
        AimMode mode = getAimMode(robotPose, feedTargetMode);
        Translation3d baseTarget = mode == AimMode.FEED
                ? getFeedTarget(robotPose, feedTargetMode)
                : BASE_HUB_TARGET;
        return AllianceFlipUtil.apply(baseTarget);
    }

    public static boolean isFeedModeActive(Pose2d robotPose) {
        return isFeedModeActive(robotPose, FeedTargetMode.DEFAULT_LEFT);
    }

    public static boolean isFeedModeActive(Pose2d robotPose, FeedTargetMode feedTargetMode) {
        if (robotPose == null) {
            return false;
        }
        return getAimMode(robotPose, feedTargetMode) == AimMode.FEED;
    }

    public static boolean isSotmEnabled() {
        return kEnableSotm;
    }

    public static double getRobotXBlueMeters(Pose2d robotPose) {
        if (robotPose == null) {
            return 0.0;
        }
        return AllianceFlipUtil.applyX(robotPose.getX());
    }

    public static void publishModeTelemetry(Pose2d robotPose) {
        SolverState nonSotm = solveNonSotm(robotPose, FeedTargetMode.DEFAULT_LEFT);
        SolverState sotm = solveSotm(robotPose, new ChassisSpeeds(), FeedTargetMode.DEFAULT_LEFT, false);
        publishSolverTelemetry(nonSotm, sotm, kEnableSotm ? sotm : nonSotm);
    }

    public static double getDistanceMeters(Pose2d robotPose) {
        return getDistanceMeters(robotPose, false);
    }

    public static double getDistanceMeters(Pose2d robotPose, boolean useRobotSideForFeedTarget) {
        return getDistanceMeters(robotPose, useRobotSideForFeedTarget
                ? FeedTargetMode.ROBOT_SIDE
                : FeedTargetMode.DEFAULT_LEFT);
    }

    public static double getDistanceMeters(Pose2d robotPose, FeedTargetMode feedTargetMode) {
        if (robotPose == null) {
            return 0.0;
        }
        Translation2d shooterExitXY = getShooterExitXY(robotPose);
        return shooterExitXY.getDistance(getTarget(robotPose, feedTargetMode).toTranslation2d());
    }

    private static final int LOOKAHEAD_ITERATIONS = 20;
    private static final double CONTROL_PHASE_DELAY_SEC = 0.03;
    private static final double SOTM_BLEND_START_MPS = 0.15;
    private static final double SOTM_BLEND_FULL_MPS = 0.60;
    private static final double NOMINAL_LOOP_DT_SEC = 0.02;
    private static final double MAX_ACCELERATION_MPS2 = 6.0;
    private static final double MAX_ACCEL_LOOKAHEAD_SEC = 0.35;
    private static final double ACCEL_FILTER_ALPHA = 0.25;
    private static Translation2d lastShooterFieldVelocity = new Translation2d();
    private static Translation2d filteredShooterFieldAcceleration = new Translation2d();
    private static boolean motionHistoryInitialized = false;

    public static ShootingSolution calculate(Pose2d robotPose, ChassisSpeeds robotSpeeds) {
        return calculate(robotPose, robotSpeeds, false);
    }

    public static ShootingSolution calculate(
            Pose2d robotPose,
            ChassisSpeeds robotSpeeds,
            boolean useRobotSideForFeedTarget) {
        return calculate(robotPose, robotSpeeds,
                useRobotSideForFeedTarget ? FeedTargetMode.ROBOT_SIDE : FeedTargetMode.DEFAULT_LEFT);
    }

    public static ShootingSolution calculate(
            Pose2d robotPose,
            ChassisSpeeds robotSpeeds,
            FeedTargetMode feedTargetMode) {
        SolverState nonSotm = solveNonSotm(robotPose, feedTargetMode);
        SolverState sotm = solveSotm(robotPose, robotSpeeds, feedTargetMode, true);
        SolverState active = kEnableSotm ? sotm : nonSotm;
        publishSolverTelemetry(nonSotm, sotm, active);
        return active.solution();
    }

    static ShootingSolution calculateNonSotm(Pose2d robotPose) {
        return calculateNonSotm(robotPose, false);
    }

    static ShootingSolution calculateNonSotm(Pose2d robotPose, boolean useRobotSideForFeedTarget) {
        return calculateNonSotm(robotPose,
                useRobotSideForFeedTarget ? FeedTargetMode.ROBOT_SIDE : FeedTargetMode.DEFAULT_LEFT);
    }

    static ShootingSolution calculateNonSotm(Pose2d robotPose, FeedTargetMode feedTargetMode) {
        return solveNonSotm(robotPose, feedTargetMode).solution();
    }

    static ShootingSolution calculateSotm(Pose2d robotPose, ChassisSpeeds robotSpeeds) {
        return calculateSotm(robotPose, robotSpeeds, false);
    }

    static ShootingSolution calculateSotm(
            Pose2d robotPose,
            ChassisSpeeds robotSpeeds,
            boolean useRobotSideForFeedTarget) {
        return calculateSotm(robotPose, robotSpeeds,
                useRobotSideForFeedTarget ? FeedTargetMode.ROBOT_SIDE : FeedTargetMode.DEFAULT_LEFT);
    }

    static ShootingSolution calculateSotm(
            Pose2d robotPose,
            ChassisSpeeds robotSpeeds,
            FeedTargetMode feedTargetMode) {
        return solveSotm(robotPose, robotSpeeds, feedTargetMode, false).solution();
    }

    private static SolverState solveNonSotm(Pose2d robotPose, FeedTargetMode feedTargetMode) {
        if (robotPose == null) {
            return invalidSolverState(AimMode.HUB, 0.0, getTarget());
        }

        SolverRequest request = createSolverRequest(robotPose, feedTargetMode);
        Translation2d aimPoint = request.fieldTarget().toTranslation2d();
        double distanceMeters = request.shooterExitXY().getDistance(aimPoint);
        double naiveAirTimeSec = sanitizeAirTimeSec(distanceMeters);
        ShotParameter shot = lookupShot(request.mode(), feedTargetMode, distanceMeters).orElse(null);

        return new SolverState(
                request.mode(),
                request.robotXBlueMeters(),
                request.fieldTarget(),
                aimPoint,
                buildSolution(
                        request.robotPose(),
                        aimPoint,
                        distanceMeters,
                        distanceMeters,
                        naiveAirTimeSec,
                        0.0,
                        0.0,
                        0.0,
                        shot));
    }

    private static SolverState solveSotm(
            Pose2d robotPose,
            ChassisSpeeds robotSpeeds,
            FeedTargetMode feedTargetMode,
            boolean updateMotionHistory) {
        if (robotPose == null || robotSpeeds == null) {
            if (updateMotionHistory) {
                resetMotionHistory();
            }
            return invalidSolverState(AimMode.HUB, 0.0, getTarget());
        }

        SotmMotionState motionState = computeSotmMotionState(robotPose, robotSpeeds, updateMotionHistory);
        SolverRequest request = createSolverRequest(motionState.predictedPose(), feedTargetMode);
        Translation2d baseTargetXY = request.fieldTarget().toTranslation2d();
        double distanceNoLookaheadMeters = request.shooterExitXY().getDistance(baseTargetXY);
        double naiveAirTimeSec = sanitizeAirTimeSec(distanceNoLookaheadMeters);
        Translation2d aimPoint = computeSotmAimPoint(request.shooterExitXY(), baseTargetXY, motionState);
        double distanceMeters = request.shooterExitXY().getDistance(aimPoint);
        ShotParameter shot = lookupShot(request.mode(), feedTargetMode, distanceMeters).orElse(null);

        return new SolverState(
                request.mode(),
                request.robotXBlueMeters(),
                request.fieldTarget(),
                aimPoint,
                buildSolution(
                        motionState.predictedPose(),
                        aimPoint,
                        distanceMeters,
                        distanceNoLookaheadMeters,
                        naiveAirTimeSec,
                        computeTargetOmega(motionState.predictedPose(), aimPoint, motionState),
                        motionState.shooterFieldSpeedMps(),
                        motionState.shooterFieldAccelMps2(),
                        shot));
    }

    private static SolverRequest createSolverRequest(Pose2d robotPose, FeedTargetMode feedTargetMode) {
        AimMode mode = getAimMode(robotPose, feedTargetMode);
        return new SolverRequest(
                robotPose,
                mode,
                getRobotXBlueMeters(robotPose),
                getTarget(robotPose, feedTargetMode),
                getShooterExitXY(robotPose));
    }

    private static SolverState invalidSolverState(AimMode mode, double robotXBlueMeters, Translation3d fieldTarget) {
        return new SolverState(
                mode,
                robotXBlueMeters,
                fieldTarget,
                fieldTarget.toTranslation2d(),
                new ShootingSolution(new Rotation2d(), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, false));
    }

    private static ShootingSolution buildSolution(
            Pose2d aimingPose,
            Translation2d aimPoint,
            double distanceMeters,
            double distanceNoLookaheadMeters,
            double naiveAirTimeSec,
            double targetOmega,
            double shooterFieldSpeedMps,
            double shooterFieldAccelMps2,
            ShotParameter shot) {
        double dx = aimPoint.getX() - aimingPose.getX();
        double dy = aimPoint.getY() - aimingPose.getY();
        Rotation2d targetRobotYaw = new Rotation2d(dx, dy);
        boolean valid = shot != null && shot.isValid();
        return new ShootingSolution(
                targetRobotYaw,
                valid ? shot.hoodCommandDeg() : 0.0,
                valid ? shot.flywheelRpm() : 0.0,
                targetOmega,
                distanceMeters,
                distanceNoLookaheadMeters,
                naiveAirTimeSec,
                shooterFieldSpeedMps,
                shooterFieldAccelMps2,
                valid);
    }

    private static SotmMotionState computeSotmMotionState(
            Pose2d robotPose,
            ChassisSpeeds robotSpeeds,
            boolean updateMotionHistory) {
        if (!kEnableSotm) {
            if (updateMotionHistory) {
                resetMotionHistory();
            }
            return new SotmMotionState(robotPose, new Translation2d(), new Translation2d(), 0.0, 0.0, 0.0);
        }

        Pose2d predictedPose = robotPose.exp(new Twist2d(
                robotSpeeds.vxMetersPerSecond * CONTROL_PHASE_DELAY_SEC,
                robotSpeeds.vyMetersPerSecond * CONTROL_PHASE_DELAY_SEC,
                robotSpeeds.omegaRadiansPerSecond * CONTROL_PHASE_DELAY_SEC));
        Translation2d shooterOffsetXY = getShooterOffsetXY(predictedPose);
        ChassisSpeeds fieldSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(robotSpeeds, predictedPose.getRotation());
        Translation2d fieldLinearVelocity = new Translation2d(
                fieldSpeeds.vxMetersPerSecond,
                fieldSpeeds.vyMetersPerSecond);
        double fieldSpeedMps = fieldLinearVelocity.getNorm();
        double sotmBlend = MathUtil.clamp(
                (fieldSpeedMps - SOTM_BLEND_START_MPS) / (SOTM_BLEND_FULL_MPS - SOTM_BLEND_START_MPS),
                0.0,
                1.0);
        Translation2d rotationalVelocityAtShooter = new Translation2d(
                -robotSpeeds.omegaRadiansPerSecond * shooterOffsetXY.getY(),
                robotSpeeds.omegaRadiansPerSecond * shooterOffsetXY.getX());
        Translation2d shooterFieldVelocity = fieldLinearVelocity.plus(rotationalVelocityAtShooter.times(sotmBlend));
        Translation2d shooterFieldAcceleration = new Translation2d();

        if (updateMotionHistory) {
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
        }

        return new SotmMotionState(
                predictedPose,
                shooterFieldVelocity,
                shooterFieldAcceleration,
                shooterFieldVelocity.getNorm(),
                shooterFieldAcceleration.getNorm(),
                sotmBlend);
    }

    private static void resetMotionHistory() {
        motionHistoryInitialized = false;
        lastShooterFieldVelocity = new Translation2d();
        filteredShooterFieldAcceleration = new Translation2d();
    }

    private static Translation2d computeSotmAimPoint(
            Translation2d shooterExitXY,
            Translation2d fieldTargetXY,
            SotmMotionState motionState) {
        Translation2d aimPoint = fieldTargetXY;
        if (!kEnableSotm) {
            return aimPoint;
        }

        for (int i = 0; i < LOOKAHEAD_ITERATIONS; i++) {
            double iterationDistanceMeters = shooterExitXY.getDistance(aimPoint);
            double airTimeSec = sanitizeAirTimeSec(iterationDistanceMeters);
            Translation2d offset = motionState.shooterFieldVelocity().times(airTimeSec * motionState.sotmBlend());
            if (motionState.sotmBlend() > 0.0) {
                double accelTimeSec = Math.min(airTimeSec, MAX_ACCEL_LOOKAHEAD_SEC);
                offset = offset.plus(motionState.shooterFieldAcceleration()
                        .times(0.5 * accelTimeSec * accelTimeSec * motionState.sotmBlend()));
            }
            aimPoint = fieldTargetXY.minus(offset);
        }
        return aimPoint;
    }

    private static double computeTargetOmega(
            Pose2d aimingPose,
            Translation2d aimPoint,
            SotmMotionState motionState) {
        if (!kEnableSotm || motionState.sotmBlend() <= 0.0) {
            return 0.0;
        }

        double yawDistanceMeters = Math.max(aimingPose.getTranslation().getDistance(aimPoint), 0.5);
        double losAngleRad = new Rotation2d(
                aimPoint.getX() - aimingPose.getX(),
                aimPoint.getY() - aimingPose.getY()).getRadians();
        return (motionState.shooterFieldVelocity().getX() * Math.sin(losAngleRad)
                - motionState.shooterFieldVelocity().getY() * Math.cos(losAngleRad)) / yawDistanceMeters
                * motionState.sotmBlend();
    }

    private static Optional<ShotParameter> lookupShot(
            AimMode mode,
            FeedTargetMode feedTargetMode,
            double distanceMeters) {
        if (mode != AimMode.FEED) {
            return ShooterInterpolationMaps.getHub(distanceMeters);
        }
        return feedTargetMode == FeedTargetMode.OVER_BUMP
                ? ShooterInterpolationMaps.getOverBumpFeed(distanceMeters)
                : ShooterInterpolationMaps.getFeed(distanceMeters);
    }

    private static double sanitizeAirTimeSec(double distanceMeters) {
        double airTimeSec = ShooterInterpolationMaps.getAirTime(distanceMeters);
        if (!Double.isFinite(airTimeSec) || airTimeSec < 0.0) {
            return 0.0;
        }
        return airTimeSec;
    }

    private static Translation2d getShooterOffsetXY(Pose2d robotPose) {
        return new Translation2d(SHOOTER_OFFSET.getX(), SHOOTER_OFFSET.getY()).rotateBy(robotPose.getRotation());
    }

    private static Translation2d getShooterExitXY(Pose2d robotPose) {
        return robotPose.getTranslation().plus(getShooterOffsetXY(robotPose));
    }

    private static Translation2d clampVectorNorm(Translation2d vector, double maxNorm) {
        double norm = vector.getNorm();
        if (norm <= maxNorm || norm <= 1e-9) {
            return vector;
        }
        return vector.times(maxNorm / norm);
    }

    private static AimMode getAimMode(Pose2d robotPose, FeedTargetMode feedTargetMode) {
        if (feedTargetMode == FeedTargetMode.OVER_BUMP) {
            return AimMode.FEED;
        }
        return AllianceFlipUtil.applyX(robotPose.getX()) > FEED_MODE_BLUE_X_THRESHOLD_M ? AimMode.FEED : AimMode.HUB;
    }

    private static Translation3d getFeedTarget(Pose2d robotPose, FeedTargetMode feedTargetMode) {
        if (feedTargetMode == FeedTargetMode.OVER_BUMP) {
            return BASE_OVER_BUMP_FEED_TARGET;
        }
        if (feedTargetMode != FeedTargetMode.ROBOT_SIDE || robotPose == null) {
            return BASE_FEED_LEFT_TARGET;
        }
        double robotYBlueMeters = AllianceFlipUtil.applyY(robotPose.getY());
        return robotYBlueMeters >= FieldConstants.LinesHorizontal.center
                ? BASE_FEED_LEFT_TARGET
                : BASE_FEED_RIGHT_TARGET;
    }

    private static void publishSolverTelemetry(SolverState nonSotm, SolverState sotm, SolverState active) {
        publishActiveSolverTelemetry(active);
        publishNonSotmSolverTelemetry(nonSotm);
        publishSotmSolverTelemetry(sotm);
    }

    private static void publishActiveSolverTelemetry(SolverState state) {
        telemetry.publishAutoAimSolver(toSnapshot(state), modeLabel(state.mode()));
        telemetry.publishAutoAimSolverGeometry(state.fieldTarget(), state.aimPoint());
    }

    private static void publishNonSotmSolverTelemetry(SolverState state) {
        telemetry.publishAutoAimNonSotmSolver(toSnapshot(state), modeLabel(state.mode()));
        telemetry.publishAutoAimNonSotmGeometry(state.fieldTarget(), state.aimPoint());
    }

    private static void publishSotmSolverTelemetry(SolverState state) {
        telemetry.publishAutoAimSotmSolver(toSnapshot(state), modeLabel(state.mode()));
        telemetry.publishAutoAimSotmGeometry(state.fieldTarget(), state.aimPoint());
    }

    private static AutoAimSolverSnapshot toSnapshot(SolverState state) {
        ShootingSolution solution = state.solution();
        return new AutoAimSolverSnapshot(
                state.mode() == AimMode.FEED ? 1 : 0,
                state.mode() == AimMode.FEED,
                state.robotXBlueMeters(),
                solution.distanceMeters(),
                solution.distanceNoLookaheadMeters(),
                solution.naiveAirTimeSec(),
                solution.isValid(),
                solution.targetRobotYaw().getRadians(),
                solution.targetOmega(),
                Math.toRadians(solution.hoodCommandDeg()),
                solution.flywheelRpm() / 60.0,
                solution.shooterFieldSpeedMps(),
                solution.shooterFieldAccelMps2());
    }

    private static String modeLabel(AimMode mode) {
        return mode == AimMode.FEED ? "Feed" : "Hub";
    }
}
