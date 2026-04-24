package com.team9470.sim;

import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.junit.jupiter.api.Assertions.fail;
import static org.junit.jupiter.api.Assumptions.assumeTrue;

import choreo.Choreo;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import com.team9470.FieldConstants;
import com.team9470.Robot;
import com.team9470.bline.FlippingUtil;
import com.team9470.subsystems.swerve.Swerve;
import com.team9470.subsystems.vision.Vision;
import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.SendableChooserSim;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.junit.jupiter.api.Test;

class AutoTimingSimulationTest {
    private static final double kLoopPeriodSec = 0.02;
    private static final int kDisabledPrerollCycles = 5;
    private static final String kEnabledProperty = "autoSim.enabled";
    private static final String kAutoProperty = "autoSim.auto";
    private static final String kTargetTrajectoryProperty = "autoSim.targetTrajectory";
    private static final String kAllianceProperty = "autoSim.alliance";
    private static final String kMirrorTargetAcrossYProperty = "autoSim.mirrorTargetAcrossY";
    private static final String kMaxTimeSecProperty = "autoSim.maxTimeSec";
    private static final String kTargetToleranceMProperty = "autoSim.targetToleranceM";
    private static final String kDisableVisionProperty = "autoSim.disableVision";
    private static final String kReferenceTrajectoryProperty = "autoSim.referenceTrajectory";
    private static final String kReferenceModeProperty = "autoSim.referenceMode";
    private static final String kReferenceWaypointIndexProperty = "autoSim.referenceWaypointIndex";
    private static final String kReferenceRadiusModeProperty = "autoSim.referenceRadiusMode";
    private static final String kAutoChooserPath = "/SmartDashboard/AutoChooser/";

    @Test
    void reportsAutoStartupDelayAndCenterArrivalTime() {
        assumeTrue(Boolean.parseBoolean(System.getProperty(kEnabledProperty, "false")),
                "Headless auto timing sim only runs when autoSim.enabled=true");

        if (!HAL.initialize(500, 0)) {
            fail("HAL failed to initialize");
        }

        DriverStationSim.resetData();
        DriverStationSim.setDsAttached(true);
        DriverStationSim.setEnabled(false);
        DriverStationSim.setAutonomous(false);
        DriverStationSim.setTest(false);
        DriverStationSim.setSendError(false);
        DriverStationSim.setMatchType(DriverStation.MatchType.Practice);
        DriverStationSim.setMatchTime(15.0);

        AllianceStationID allianceStation = parseAllianceStation(
                System.getProperty(kAllianceProperty, "blue"));
        DriverStationSim.setAllianceStationId(allianceStation);
        DriverStationSim.notifyNewData();

        HeadlessRobot robot = new HeadlessRobot();
        robot.robotInit();
        robot.simulationInit();

        String autoName = System.getProperty(kAutoProperty, "leftTrench");
        String targetTrajectoryName = System.getProperty(kTargetTrajectoryProperty, "leftTrenchToCenter");
        boolean mirrorTargetAcrossY = Boolean.parseBoolean(
                System.getProperty(kMirrorTargetAcrossYProperty, "false"));
        double maxTimeSec = Double.parseDouble(System.getProperty(kMaxTimeSecProperty, "15.0"));
        double targetToleranceM = Double.parseDouble(System.getProperty(kTargetToleranceMProperty, "0.15"));
        boolean disableVision = Boolean.parseBoolean(System.getProperty(kDisableVisionProperty, "false"));
        String referenceTrajectoryName = System.getProperty(kReferenceTrajectoryProperty, "");
        String referenceMode = System.getProperty(kReferenceModeProperty, "centerlineTouch");
        int referenceWaypointIndex = Integer.parseInt(System.getProperty(kReferenceWaypointIndexProperty, "-1"));
        String referenceRadiusMode = System.getProperty(kReferenceRadiusModeProperty, "side");

        try (robot; var chooser = new SendableChooserSim(kAutoChooserPath)) {
            Vision.getInstance().setVisionDisabled(disableVision);
            chooser.setSelected(autoName);
            NetworkTableInstance.getDefault().flushLocal();

            for (int i = 0; i < kDisabledPrerollCycles; i++) {
                runLoop(robot);
            }

            Pose2d targetPose = loadTargetPose(targetTrajectoryName, isRedAlliance(allianceStation), mirrorTargetAcrossY);
            double centerReachedTimestampSec = Double.NaN;
            double liveCenterlineTouchTimestampSec = Double.NaN;
            Pose2d liveCenterlineTouchPose = Pose2d.kZero;
            double lastTimestampSec = Timer.getTimestamp();
            PathReferenceData referenceData = loadReferenceData(referenceTrajectoryName);

            DriverStationSim.setAutonomous(true);
            DriverStationSim.setEnabled(true);

            Pose2d previousPose = Swerve.getInstance().getPose();
            double previousTimestampSec = Timer.getTimestamp();
            double enableTimestampSec = previousTimestampSec;
            double maxPoseX = previousPose.getX();
            boolean debugPose = true;
            int maxCycles = (int) Math.ceil(maxTimeSec / kLoopPeriodSec);
            for (int cycle = 0; cycle < maxCycles; cycle++) {
                runLoop(robot);
                lastTimestampSec = Timer.getTimestamp();

                Pose2d currentPose = Swerve.getInstance().getPose();
                maxPoseX = Math.max(maxPoseX, currentPose.getX());
                if (debugPose && cycle % 10 == 0) {
                    var requiring = CommandScheduler.getInstance().requiring(Swerve.getInstance());
                    System.out.printf(
                            "    pose t=%.3f x=%.3f y=%.3f heading=%.1f maxX=%.3f swerveCmd=%s%n",
                            lastTimestampSec - enableTimestampSec,
                            currentPose.getX(),
                            currentPose.getY(),
                            currentPose.getRotation().getDegrees(),
                            maxPoseX,
                            requiring == null ? "<none>" : requiring.getName());
                }
                if (!Double.isFinite(liveCenterlineTouchTimestampSec) && referenceData != null) {
                    CrossingMeasurement centerlineTouch =
                            measureCenterlineTouch(previousPose, currentPose, previousTimestampSec, lastTimestampSec, referenceData);
                    if (centerlineTouch.crossed) {
                        liveCenterlineTouchTimestampSec = centerlineTouch.timestampSec;
                        liveCenterlineTouchPose = centerlineTouch.pose;
                    }
                }
                if (!Double.isFinite(centerReachedTimestampSec)
                        && currentPose.getTranslation().getDistance(targetPose.getTranslation()) <= targetToleranceM) {
                    centerReachedTimestampSec = lastTimestampSec;
                    break;
                }

                previousPose = currentPose;
                previousTimestampSec = lastTimestampSec;
            }

            StartupMetrics startupMetrics = readStartupMetrics();
            AutoPathTelemetry autoPathTelemetry = readAutoPathTelemetry();
            double centerArrivalSec = Double.isFinite(centerReachedTimestampSec) && Double.isFinite(startupMetrics.autoInitTimestampSec)
                    ? centerReachedTimestampSec - startupMetrics.autoInitTimestampSec
                    : Double.NaN;
            double autoInitToLiveCenterlineTouchSec =
                    Double.isFinite(liveCenterlineTouchTimestampSec) && Double.isFinite(startupMetrics.autoInitTimestampSec)
                            ? liveCenterlineTouchTimestampSec - startupMetrics.autoInitTimestampSec
                            : Double.NaN;
            PlannedReferenceTiming plannedReferenceTiming = computePlannedReferenceTiming(
                    referenceTrajectoryName,
                    referenceMode,
                    referenceWaypointIndex,
                    referenceRadiusMode,
                    isRedAlliance(allianceStation),
                    referenceData);
            double estimatedAutoInitToReferenceSec =
                    Double.isFinite(startupMetrics.autoInitToFirstMotionSec)
                            && Double.isFinite(plannedReferenceTiming.plannedTimeSec)
                                    ? startupMetrics.autoInitToFirstMotionSec + plannedReferenceTiming.plannedTimeSec
                                    : Double.NaN;
            Pose2d finalPose = Swerve.getInstance().getPose();
            double finalTargetErrorM = finalPose.getTranslation().getDistance(targetPose.getTranslation());

            System.out.printf(
                    "%nAuto timing report%n"
                            + "  auto: %s%n"
                            + "  target trajectory: %s%n"
                            + "  alliance: %s%n"
                            + "  vision disabled: %s%n"
                            + "  auto init -> command scheduled: %.3f s%n"
                            + "  auto init -> selected command start: %.3f s%n"
                            + "  auto init -> routine first poll: %.3f s%n"
                            + "  auto init -> routine start trigger: %.3f s%n"
                            + "  auto init -> sequence start: %.3f s%n"
                            + "  auto init -> reset odometry: %.3f s%n"
                            + "  auto init -> deploy intake: %.3f s%n"
                            + "  auto init -> first trajectory init: %.3f s%n"
                            + "  auto init -> first path sample: %.3f s%n"
                            + "  auto init -> first motion command: %.3f s%n"
                            + "  auto init -> first measured movement: %.3f s%n"
                            + "  command scheduled -> selected command start: %.3f s%n"
                            + "  selected command start -> routine first poll: %.3f s%n"
                            + "  routine first poll -> routine start trigger: %.3f s%n"
                            + "  routine start trigger -> sequence start: %.3f s%n"
                            + "  sequence start -> reset odometry: %.3f s%n"
                            + "  reset odometry -> deploy intake: %.3f s%n"
                            + "  deploy intake -> first trajectory init: %.3f s%n"
                            + "  first trajectory init -> first path sample: %.3f s%n"
                            + "  first motion command -> first measured movement: %.3f s%n"
                            + "  auto init -> runtime centerline touch: %.3f s%n"
                            + "  auto init -> live odom centerline touch: %.3f s%n"
                            + "  auto init -> center reached: %.3f s%n"
                            + "  planned first-path reference: %s%n"
                            + "  path start -> reference: %.3f s%n"
                            + "  estimated auto init -> reference: %.3f s%n"
                            + "  reference pose: (%.3f, %.3f, %.1f deg)%n"
                            + "  runtime centerline touch pose: (%.3f, %.3f, %.1f deg)%n"
                            + "  live odom centerline touch pose: (%.3f, %.3f, %.1f deg)%n"
                            + "  last auto path sample timestamp: %.3f s%n"
                            + "  last auto pose error norm: %.3f m%n"
                            + "  last auto heading error: %.3f rad%n"
                            + "  first motion translation: %.3f m/s%n"
                            + "  first motion omega: %.3f rad/s%n"
                            + "  first measured movement translation: %.3f m/s%n"
                            + "  first measured movement omega: %.3f rad/s%n"
                            + "  final pose: (%.3f, %.3f, %.1f deg)%n"
                            + "  target pose: (%.3f, %.3f, %.1f deg)%n"
                            + "  final target error: %.3f m%n"
                            + "  sim stop time: %.3f s%n",
                    autoName,
                    targetTrajectoryName,
                    allianceStation,
                    disableVision,
                    startupMetrics.autoInitToCommandScheduleSec,
                    startupMetrics.autoInitToSelectedCommandStartSec,
                    startupMetrics.autoInitToRoutineFirstPollSec,
                    startupMetrics.autoInitToRoutineStartTriggerSec,
                    startupMetrics.autoInitToSequenceStartSec,
                    startupMetrics.autoInitToResetOdometrySec,
                    startupMetrics.autoInitToDeployIntakeSec,
                    startupMetrics.autoInitToFirstTrajectoryCommandInitSec,
                    startupMetrics.autoInitToFirstPathSampleSec,
                    startupMetrics.autoInitToFirstMotionSec,
                    startupMetrics.autoInitToFirstMeasuredMovementSec,
                    stageDelta(startupMetrics.autoInitToCommandScheduleSec, startupMetrics.autoInitToSelectedCommandStartSec),
                    stageDelta(startupMetrics.autoInitToSelectedCommandStartSec, startupMetrics.autoInitToRoutineFirstPollSec),
                    stageDelta(startupMetrics.autoInitToRoutineFirstPollSec, startupMetrics.autoInitToRoutineStartTriggerSec),
                    stageDelta(startupMetrics.autoInitToRoutineStartTriggerSec, startupMetrics.autoInitToSequenceStartSec),
                    stageDelta(startupMetrics.autoInitToSequenceStartSec, startupMetrics.autoInitToResetOdometrySec),
                    stageDelta(startupMetrics.autoInitToResetOdometrySec, startupMetrics.autoInitToDeployIntakeSec),
                    stageDelta(startupMetrics.autoInitToDeployIntakeSec, startupMetrics.autoInitToFirstTrajectoryCommandInitSec),
                    stageDelta(startupMetrics.autoInitToFirstTrajectoryCommandInitSec, startupMetrics.autoInitToFirstPathSampleSec),
                    stageDelta(startupMetrics.autoInitToFirstMotionSec, startupMetrics.autoInitToFirstMeasuredMovementSec),
                    startupMetrics.autoInitToCenterlineTouchSec,
                    autoInitToLiveCenterlineTouchSec,
                    centerArrivalSec,
                    plannedReferenceTiming.description,
                    plannedReferenceTiming.plannedTimeSec,
                    estimatedAutoInitToReferenceSec,
                    plannedReferenceTiming.referencePose.getX(),
                    plannedReferenceTiming.referencePose.getY(),
                    plannedReferenceTiming.referencePose.getRotation().getDegrees(),
                    startupMetrics.centerlineTouchPose.getX(),
                    startupMetrics.centerlineTouchPose.getY(),
                    startupMetrics.centerlineTouchPose.getRotation().getDegrees(),
                    liveCenterlineTouchPose.getX(),
                    liveCenterlineTouchPose.getY(),
                    liveCenterlineTouchPose.getRotation().getDegrees(),
                    autoPathTelemetry.lastSampleTimestampSec,
                    autoPathTelemetry.poseErrorNormM,
                    autoPathTelemetry.headingErrorRad,
                    startupMetrics.firstMotionTranslationMps,
                    startupMetrics.firstMotionOmegaRadPerSec,
                    startupMetrics.firstMeasuredMovementTranslationMps,
                    startupMetrics.firstMeasuredMovementOmegaRadPerSec,
                    finalPose.getX(),
                    finalPose.getY(),
                    finalPose.getRotation().getDegrees(),
                    targetPose.getX(),
                    targetPose.getY(),
                    targetPose.getRotation().getDegrees(),
                    finalTargetErrorM,
                    lastTimestampSec);

            assertTrue(Double.isFinite(startupMetrics.autoInitToFirstPathSampleSec),
                    "Auto never produced a path sample in simulation");
            assertTrue(Double.isFinite(startupMetrics.autoInitToFirstMotionSec),
                    "Auto never produced a meaningful motion command in simulation");
            if (!Double.isFinite(centerArrivalSec)) {
                System.out.printf(
                        "  note: target was not reached within %.3f m tolerance; final error %.3f m%n",
                        targetToleranceM,
                        finalTargetErrorM);
            }
        } finally {
            DriverStationSim.setEnabled(false);
            DriverStationSim.setAutonomous(false);
            DriverStationSim.setSendError(true);
            DriverStationSim.notifyNewData();
            CommandScheduler.getInstance().cancelAll();
            CommandScheduler.getInstance().run();
            HAL.shutdown();
        }
    }

    private static void runLoop(HeadlessRobot robot) {
        DriverStationSim.notifyNewData();
        robot.runLoop();
        try {
            Thread.sleep((long) Math.round(kLoopPeriodSec * 1000.0));
        } catch (InterruptedException ex) {
            Thread.currentThread().interrupt();
            throw new RuntimeException("Interrupted while waiting for real-time simulation loop", ex);
        }
    }

    private static Pose2d loadTargetPose(
            String trajectoryName,
            boolean mirrorForRedAlliance,
            boolean mirrorAcrossY) {
        Trajectory<SwerveSample> trajectory = Choreo.<SwerveSample>loadTrajectory(trajectoryName)
                .orElseThrow(() -> new IllegalStateException("Could not load target trajectory " + trajectoryName));
        Pose2d pose = trajectory.getFinalPose(mirrorForRedAlliance)
                .orElseThrow(() -> new IllegalStateException("Target trajectory has no final pose: " + trajectoryName));
        return mirrorAcrossY ? FlippingUtil.mirrorFieldPose(pose) : pose;
    }

    private static AllianceStationID parseAllianceStation(String allianceValue) {
        return switch (allianceValue.trim().toLowerCase()) {
            case "red", "red1" -> AllianceStationID.Red1;
            case "red2" -> AllianceStationID.Red2;
            case "red3" -> AllianceStationID.Red3;
            case "blue2" -> AllianceStationID.Blue2;
            case "blue3" -> AllianceStationID.Blue3;
            case "blue", "blue1" -> AllianceStationID.Blue1;
            default -> throw new IllegalArgumentException("Unsupported alliance value: " + allianceValue);
        };
    }

    private static boolean isRedAlliance(AllianceStationID station) {
        return station == AllianceStationID.Red1
                || station == AllianceStationID.Red2
                || station == AllianceStationID.Red3;
    }

    private static StartupMetrics readStartupMetrics() {
        NetworkTable startupTable = NetworkTableInstance.getDefault()
                .getTable("Telemetry")
                .getSubTable("Drive")
                .getSubTable("Auto")
                .getSubTable("Startup");
        return new StartupMetrics(
                startupTable.getEntry("AutoInitTimestampSec").getDouble(Double.NaN),
                startupTable.getEntry("AutoInitToCommandScheduleSec").getDouble(Double.NaN),
                startupTable.getEntry("AutoInitToSelectedCommandStartSec").getDouble(Double.NaN),
                startupTable.getEntry("AutoInitToRoutineFirstPollSec").getDouble(Double.NaN),
                startupTable.getEntry("AutoInitToRoutineStartTriggerSec").getDouble(Double.NaN),
                startupTable.getEntry("AutoInitToSequenceStartSec").getDouble(Double.NaN),
                startupTable.getEntry("AutoInitToResetOdometrySec").getDouble(Double.NaN),
                startupTable.getEntry("AutoInitToDeployIntakeSec").getDouble(Double.NaN),
                startupTable.getEntry("AutoInitToFirstTrajectoryCommandInitSec").getDouble(Double.NaN),
                startupTable.getEntry("AutoInitToFirstPathSampleSec").getDouble(Double.NaN),
                startupTable.getEntry("AutoInitToFirstMotionSec").getDouble(Double.NaN),
                startupTable.getEntry("AutoInitToFirstMeasuredMovementSec").getDouble(Double.NaN),
                startupTable.getEntry("AutoInitToCenterlineTouchSec").getDouble(Double.NaN),
                new Pose2d(
                        startupTable.getEntry("CenterlineTouchXM").getDouble(0.0),
                        startupTable.getEntry("CenterlineTouchYM").getDouble(0.0),
                        edu.wpi.first.math.geometry.Rotation2d.fromDegrees(
                                startupTable.getEntry("CenterlineTouchHeadingDeg").getDouble(0.0))),
                startupTable.getEntry("FirstMotionTranslationMps").getDouble(Double.NaN),
                startupTable.getEntry("FirstMotionOmegaRadPerSec").getDouble(Double.NaN),
                startupTable.getEntry("FirstMeasuredMovementTranslationMps").getDouble(Double.NaN),
                startupTable.getEntry("FirstMeasuredMovementOmegaRadPerSec").getDouble(Double.NaN));
    }

    private static AutoPathTelemetry readAutoPathTelemetry() {
        NetworkTable autoTable = NetworkTableInstance.getDefault()
                .getTable("Telemetry")
                .getSubTable("Drive")
                .getSubTable("Auto");
        return new AutoPathTelemetry(
                autoTable.getEntry("LastSampleTimestampSec").getDouble(Double.NaN),
                autoTable.getEntry("PoseErrorNormM").getDouble(Double.NaN),
                autoTable.getEntry("HeadingErrorRad").getDouble(Double.NaN));
    }

    private static PathReferenceData loadReferenceData(String trajectoryName) {
        if (trajectoryName == null || trajectoryName.isBlank()) {
            return null;
        }
        return PathReferenceData.load(trajectoryName);
    }

    private static PlannedReferenceTiming computePlannedReferenceTiming(
            String trajectoryName,
            String referenceMode,
            int waypointIndex,
            String radiusMode,
            boolean mirrorForRedAlliance,
            PathReferenceData referenceData) {
        if (trajectoryName == null || trajectoryName.isBlank()) {
            return PlannedReferenceTiming.unavailable();
        }

        Trajectory<SwerveSample> trajectory = Choreo.<SwerveSample>loadTrajectory(trajectoryName)
                .orElseThrow(() -> new IllegalStateException("Could not load reference trajectory " + trajectoryName));

        return switch (referenceMode.trim().toLowerCase()) {
            case "centerlinetouch", "centerline-touch", "touch", "bumpertouch", "bumper-touch" ->
                computeCenterlineTouchTiming(trajectoryName, trajectory, mirrorForRedAlliance, referenceData);
            case "waypoint", "waypoint-zone", "zone" ->
                computeWaypointZoneTiming(trajectoryName, trajectory, waypointIndex, radiusMode, mirrorForRedAlliance, referenceData);
            default -> throw new IllegalArgumentException("Unsupported reference mode: " + referenceMode);
        };
    }

    private record StartupMetrics(
            double autoInitTimestampSec,
            double autoInitToCommandScheduleSec,
            double autoInitToSelectedCommandStartSec,
            double autoInitToRoutineFirstPollSec,
            double autoInitToRoutineStartTriggerSec,
            double autoInitToSequenceStartSec,
            double autoInitToResetOdometrySec,
            double autoInitToDeployIntakeSec,
            double autoInitToFirstTrajectoryCommandInitSec,
            double autoInitToFirstPathSampleSec,
            double autoInitToFirstMotionSec,
            double autoInitToFirstMeasuredMovementSec,
            double autoInitToCenterlineTouchSec,
            Pose2d centerlineTouchPose,
            double firstMotionTranslationMps,
            double firstMotionOmegaRadPerSec,
            double firstMeasuredMovementTranslationMps,
            double firstMeasuredMovementOmegaRadPerSec) {
    }

    private static double stageDelta(double startSec, double endSec) {
        if (!Double.isFinite(startSec) || !Double.isFinite(endSec)) {
            return Double.NaN;
        }
        return endSec - startSec;
    }

    private record AutoPathTelemetry(
            double lastSampleTimestampSec,
            double poseErrorNormM,
            double headingErrorRad) {
    }

    private static PlannedReferenceTiming computeCenterlineTouchTiming(
            String trajectoryName,
            Trajectory<SwerveSample> trajectory,
            boolean mirrorForRedAlliance,
            PathReferenceData referenceData) {
        if (referenceData == null) {
            return PlannedReferenceTiming.unavailable();
        }
        double touchThresholdX = FieldConstants.LinesVertical.center - referenceData.centerlineTouchOffsetMeters();
        SwerveSample previous = null;
        double previousDx = Double.NaN;

        for (SwerveSample sample : trajectory.samples()) {
            SwerveSample effectiveSample = mirrorForRedAlliance ? sample.flipped() : sample;
            double dx = effectiveSample.x - touchThresholdX;
            if (previous == null) {
                previous = effectiveSample;
                previousDx = dx;
                if (Math.abs(dx) < 1e-9) {
                    return new PlannedReferenceTiming(
                            "%s centerline touch at robot-center X %.3f m".formatted(trajectoryName, touchThresholdX),
                            effectiveSample.getTimestamp(),
                            effectiveSample.getPose());
                }
                continue;
            }

            if (Math.abs(dx) < 1e-9 || Math.signum(dx) != Math.signum(previousDx)) {
                Pose2d crossingPose = interpolatePoseAtX(previous, effectiveSample, touchThresholdX);
                double crossingTime = interpolateAtX(previous, effectiveSample, touchThresholdX,
                        previous.getTimestamp(), effectiveSample.getTimestamp());
                return new PlannedReferenceTiming(
                        "%s centerline touch at robot-center X %.3f m".formatted(trajectoryName, touchThresholdX),
                        crossingTime,
                        crossingPose);
            }

            previous = effectiveSample;
            previousDx = dx;
        }

        return new PlannedReferenceTiming(
                "%s centerline touch at robot-center X %.3f m (not reached)".formatted(trajectoryName, touchThresholdX),
                Double.NaN,
                Pose2d.kZero);
    }

    private static PlannedReferenceTiming computeWaypointZoneTiming(
            String trajectoryName,
            Trajectory<SwerveSample> trajectory,
            int waypointIndex,
            String radiusMode,
            boolean mirrorForRedAlliance,
            PathReferenceData referenceData) {
        if (waypointIndex < 0) {
            return PlannedReferenceTiming.unavailable();
        }

        if (referenceData == null) {
            return PlannedReferenceTiming.unavailable();
        }
        Pose2d waypointPose = referenceData.waypointPose(waypointIndex, mirrorForRedAlliance);
        double radiusMeters = referenceData.radiusMeters(radiusMode);

        SwerveSample previous = null;
        double previousDistance = Double.NaN;
        for (SwerveSample sample : trajectory.samples()) {
            SwerveSample effectiveSample = mirrorForRedAlliance ? sample.flipped() : sample;
            double distance = effectiveSample.getPose().getTranslation()
                    .getDistance(waypointPose.getTranslation());
            if (distance <= radiusMeters) {
                if (previous == null || !Double.isFinite(previousDistance)) {
                    return new PlannedReferenceTiming(
                            "%s waypoint %d within %.3f m (%s)".formatted(
                                    trajectoryName,
                                    waypointIndex,
                                    radiusMeters,
                                    radiusMode),
                            effectiveSample.getTimestamp(),
                            effectiveSample.getPose());
                }

                double interpolationDenominator = distance - previousDistance;
                double fraction = Math.abs(interpolationDenominator) < 1e-9
                        ? 1.0
                        : (radiusMeters - previousDistance) / interpolationDenominator;
                fraction = Math.max(0.0, Math.min(1.0, fraction));
                return new PlannedReferenceTiming(
                        "%s waypoint %d within %.3f m (%s)".formatted(
                                trajectoryName,
                                waypointIndex,
                                radiusMeters,
                                radiusMode),
                        interpolate(previous.getTimestamp(), effectiveSample.getTimestamp(), fraction),
                        interpolatePose(previous.getPose(), effectiveSample.getPose(), fraction));
            }

            previous = effectiveSample;
            previousDistance = distance;
        }

        return new PlannedReferenceTiming(
                "%s waypoint %d within %.3f m (%s) (not reached)".formatted(
                        trajectoryName,
                        waypointIndex,
                        radiusMeters,
                        radiusMode),
                Double.NaN,
                Pose2d.kZero);
    }

    private static Pose2d interpolatePoseAtX(SwerveSample start, SwerveSample end, double targetX) {
        double fraction = interpolationFractionForX(start, end, targetX);
        return interpolatePose(start.getPose(), end.getPose(), fraction);
    }

    private static double interpolateAtX(SwerveSample start, SwerveSample end, double targetX, double startValue, double endValue) {
        return interpolate(startValue, endValue, interpolationFractionForX(start, end, targetX));
    }

    private static double interpolationFractionForX(SwerveSample start, SwerveSample end, double targetX) {
        double denominator = end.x - start.x;
        if (Math.abs(denominator) < 1e-9) {
            return 0.0;
        }
        return Math.max(0.0, Math.min(1.0, (targetX - start.x) / denominator));
    }

    private static Pose2d interpolatePose(Pose2d start, Pose2d end, double fraction) {
        return new Pose2d(
                interpolate(start.getX(), end.getX(), fraction),
                interpolate(start.getY(), end.getY(), fraction),
                start.getRotation().interpolate(end.getRotation(), fraction));
    }

    private static double interpolate(double start, double end, double fraction) {
        return start + (end - start) * fraction;
    }

    private static CrossingMeasurement measureCenterlineTouch(
            Pose2d startPose,
            Pose2d endPose,
            double startTimestampSec,
            double endTimestampSec,
            PathReferenceData referenceData) {
        double targetX = FieldConstants.LinesVertical.center - referenceData.centerlineTouchOffsetMeters();
        double startDx = startPose.getX() - targetX;
        double endDx = endPose.getX() - targetX;
        if (Math.abs(startDx) < 1e-9) {
            return new CrossingMeasurement(true, startTimestampSec, startPose);
        }
        if (Math.abs(endDx) < 1e-9 || Math.signum(startDx) != Math.signum(endDx)) {
            double fraction = interpolationFraction(startPose.getX(), endPose.getX(), targetX);
            return new CrossingMeasurement(
                    true,
                    interpolate(startTimestampSec, endTimestampSec, fraction),
                    interpolatePose(startPose, endPose, fraction));
        }
        return new CrossingMeasurement(false, Double.NaN, Pose2d.kZero);
    }

    private static double interpolationFraction(double start, double end, double target) {
        double denominator = end - start;
        if (Math.abs(denominator) < 1e-9) {
            return 0.0;
        }
        return Math.max(0.0, Math.min(1.0, (target - start) / denominator));
    }

    private record PlannedReferenceTiming(
            String description,
            double plannedTimeSec,
            Pose2d referencePose) {
        static PlannedReferenceTiming unavailable() {
            return new PlannedReferenceTiming("unavailable", Double.NaN, Pose2d.kZero);
        }
    }

    private record CrossingMeasurement(
            boolean crossed,
            double timestampSec,
            Pose2d pose) {
    }

    private record PathReferenceData(
            java.util.List<Translation2d> waypoints,
            double frontBumperMeters,
            double sideBumperMeters,
            double backBumperMeters) {
        static PathReferenceData load(String trajectoryName) {
            java.nio.file.Path file = java.nio.file.Path.of("src/main/deploy/choreo", trajectoryName + ".traj");
            try {
                String json = java.nio.file.Files.readString(file);
                java.util.ArrayList<Translation2d> loadedWaypoints = new java.util.ArrayList<>();
                java.util.regex.Matcher waypointMatcher = java.util.regex.Pattern
                        .compile("\\{\"x\":([0-9eE+\\-.]+),\\s*\"y\":([0-9eE+\\-.]+),\\s*\"heading\":")
                        .matcher(json);
                while (waypointMatcher.find()) {
                    loadedWaypoints.add(new Translation2d(
                            Double.parseDouble(waypointMatcher.group(1)),
                            Double.parseDouble(waypointMatcher.group(2))));
                }

                double front = extractBumperValue(json, "front");
                double side = extractBumperValue(json, "side");
                double back = extractBumperValue(json, "back");
                return new PathReferenceData(loadedWaypoints, front, side, back);
            } catch (java.io.IOException ex) {
                throw new RuntimeException("Failed to read reference trajectory file for " + trajectoryName, ex);
            }
        }

        Pose2d waypointPose(int index, boolean mirrorForRedAlliance) {
            if (index < 0 || index >= waypoints.size()) {
                throw new IllegalArgumentException(
                        "Waypoint index " + index + " is out of range for reference trajectory; found " + waypoints.size());
            }
            Pose2d basePose = new Pose2d(waypoints.get(index), edu.wpi.first.math.geometry.Rotation2d.kZero);
            return mirrorForRedAlliance ? FlippingUtil.flipFieldPose(basePose) : basePose;
        }

        double radiusMeters(String mode) {
            return switch (mode.trim().toLowerCase()) {
                case "front" -> frontBumperMeters;
                case "back" -> backBumperMeters;
                case "circumscribed", "corner", "diagonal" -> Math.hypot(Math.max(frontBumperMeters, backBumperMeters), sideBumperMeters);
                case "side", "width" -> sideBumperMeters;
                default -> throw new IllegalArgumentException("Unsupported reference radius mode: " + mode);
            };
        }

        double centerlineTouchOffsetMeters() {
            return sideBumperMeters;
        }

        private static double extractBumperValue(String json, String key) {
            java.util.regex.Matcher matcher = java.util.regex.Pattern
                    .compile("\"" + key + "\"\\s*:\\s*([0-9eE+\\-.]+)")
                    .matcher(json);
            if (!matcher.find()) {
                throw new IllegalStateException("Could not find bumper value for " + key);
            }
            return Double.parseDouble(matcher.group(1));
        }
    }

    private static final class HeadlessRobot extends Robot {
        void runLoop() {
            super.loopFunc();
        }
    }
}
