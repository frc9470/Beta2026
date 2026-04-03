package com.team9470.telemetry;

import com.team9470.telemetry.structs.AutoAimSolverSnapshot;
import com.team9470.telemetry.structs.DriveStatusSnapshot;
import com.team9470.telemetry.structs.HopperSnapshot;
import com.team9470.telemetry.structs.HopperPreloadSnapshot;
import com.team9470.telemetry.structs.IntakeSnapshot;
import com.team9470.telemetry.structs.PracticeTimerSnapshot;
import com.team9470.telemetry.structs.ShooterCharacterizationSnapshot;
import com.team9470.telemetry.structs.ShooterSnapshot;
import com.team9470.telemetry.structs.SimSnapshot;
import com.team9470.telemetry.structs.SuperstructureSnapshot;
import com.team9470.telemetry.structs.TimedShotSnapshot;
import com.team9470.telemetry.structs.VisionCameraSnapshot;
import com.team9470.telemetry.structs.VisionSnapshot;
import com.team9470.telemetry.structs.YShotSnapshot;
import com.team9470.util.TelemetryUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.networktables.StringPublisher;
import java.util.HashMap;
import java.util.Map;

public final class TelemetryManager {
    public static final int VALIDATION_OK = 0;
    public static final int VALIDATION_OUTSIDE_FIELD = 1;
    public static final int VALIDATION_MAX_VELOCITY = 2;
    public static final int VALIDATION_MAX_CORRECTION = 3;

    private static final TelemetryManager instance = new TelemetryManager();

    public static TelemetryManager getInstance() {
        return instance;
    }

    private final NetworkTable telemetryTable = NetworkTableInstance.getDefault().getTable("Telemetry");

    private final NetworkTable driveTable = telemetryTable.getSubTable("Drive");
    private final StructPublisher<Pose2d> drivePosePublisher = driveTable.getStructTopic("Pose", Pose2d.struct).publish();
    private final StructPublisher<ChassisSpeeds> driveSpeedsPublisher = driveTable.getStructTopic("Speeds", ChassisSpeeds.struct)
            .publish();
    private final StructArrayPublisher<SwerveModuleState> driveModuleStatesPublisher = driveTable
            .getStructArrayTopic("Modules/States", SwerveModuleState.struct).publish();
    private final StructArrayPublisher<SwerveModuleState> driveModuleTargetsPublisher = driveTable
            .getStructArrayTopic("Modules/Targets", SwerveModuleState.struct).publish();
    private final StructArrayPublisher<SwerveModulePosition> driveModulePositionsPublisher = driveTable
            .getStructArrayTopic("Modules/Positions", SwerveModulePosition.struct).publish();
    private final DoubleArrayPublisher driveModuleVelocityPublisher = driveTable
            .getDoubleArrayTopic("Modules/DriveVelocityRps").publish();
    private final DoubleArrayPublisher driveModuleStatorCurrentPublisher = driveTable
            .getDoubleArrayTopic("Modules/DriveStatorCurrentAmps").publish();
    private final StructPublisher<DriveStatusSnapshot> driveStatusPublisher = driveTable
            .getStructTopic("Status", DriveStatusSnapshot.struct).publish();
    private final NetworkTable driveAutoTable = driveTable.getSubTable("Auto");
    private final BooleanPublisher driveAutoActivePublisher = driveAutoTable.getBooleanTopic("Active").publish();
    private final DoublePublisher driveAutoLastSampleTimestampPublisher = TelemetryUtil.publishDouble(
            driveAutoTable, "LastSampleTimestampSec", "s");
    private final StructPublisher<Pose2d> driveAutoDesiredPosePublisher = driveAutoTable
            .getStructTopic("DesiredPose", Pose2d.struct).publish();
    private final StructPublisher<Pose2d> driveAutoMeasuredPosePublisher = driveAutoTable
            .getStructTopic("MeasuredPose", Pose2d.struct).publish();
    private final StructPublisher<Translation2d> driveAutoPoseErrorPublisher = driveAutoTable
            .getStructTopic("PoseError", Translation2d.struct).publish();
    private final DoublePublisher driveAutoPoseErrorNormPublisher = TelemetryUtil.publishDouble(
            driveAutoTable, "PoseErrorNormM", "m");
    private final DoublePublisher driveAutoHeadingErrorPublisher = TelemetryUtil.publishDouble(
            driveAutoTable, "HeadingErrorRad", "rad");
    private final StructPublisher<ChassisSpeeds> driveAutoFeedforwardSpeedsPublisher = driveAutoTable
            .getStructTopic("FeedforwardSpeeds", ChassisSpeeds.struct).publish();
    private final StructPublisher<ChassisSpeeds> driveAutoFeedbackSpeedsPublisher = driveAutoTable
            .getStructTopic("FeedbackSpeeds", ChassisSpeeds.struct).publish();
    private final StructPublisher<ChassisSpeeds> driveAutoCommandedSpeedsPublisher = driveAutoTable
            .getStructTopic("CommandedSpeeds", ChassisSpeeds.struct).publish();
    private final StructPublisher<ChassisSpeeds> driveAutoMeasuredSpeedsPublisher = driveAutoTable
            .getStructTopic("MeasuredSpeeds", ChassisSpeeds.struct).publish();
    private final DoubleArrayPublisher driveAutoModuleForcesXPublisher = driveAutoTable.getDoubleArrayTopic("ModuleForcesX")
            .publish();
    private final DoubleArrayPublisher driveAutoModuleForcesYPublisher = driveAutoTable.getDoubleArrayTopic("ModuleForcesY")
            .publish();

    private final NetworkTable intakeTable = telemetryTable.getSubTable("Intake");
    private final StructPublisher<IntakeSnapshot> intakeStatePublisher = intakeTable
            .getStructTopic("State", IntakeSnapshot.struct).publish();
    private final NetworkTable intakePivotTable = intakeTable.getSubTable("Pivot");
    private final DoublePublisher intakePivotStatorCurrentPublisher = TelemetryUtil.publishDouble(
            intakePivotTable, "StatorCurrentAmps", "A");
    private final NetworkTable intakeLeftRollerTable = intakeTable.getSubTable("LeftRoller");
    private final DoublePublisher intakeLeftRollerStatorCurrentPublisher = TelemetryUtil.publishDouble(
            intakeLeftRollerTable, "StatorCurrentAmps", "A");
    private final NetworkTable intakeRightRollerTable = intakeTable.getSubTable("RightRoller");
    private final DoublePublisher intakeRightRollerStatorCurrentPublisher = TelemetryUtil.publishDouble(
            intakeRightRollerTable, "StatorCurrentAmps", "A");

    private final NetworkTable shooterTable = telemetryTable.getSubTable("Shooter");
    private final StructPublisher<ShooterSnapshot> shooterStatePublisher = shooterTable
            .getStructTopic("State", ShooterSnapshot.struct).publish();
    private final StructPublisher<AutoAimSolverSnapshot> shooterSolverPublisher = shooterTable
            .getStructTopic("Solver/State", AutoAimSolverSnapshot.struct).publish();
    private final NetworkTable shooterCharacterizationTable = shooterTable.getSubTable("Characterization");
    private final StructPublisher<ShooterCharacterizationSnapshot> shooterCharacterizationPublisher = shooterCharacterizationTable
            .getStructTopic("State", ShooterCharacterizationSnapshot.struct).publish();
    private final BooleanPublisher shooterCharacterizationActivePublisher = shooterCharacterizationTable
            .getBooleanTopic("Active").publish();
    private final BooleanPublisher shooterCharacterizationCompletePublisher = shooterCharacterizationTable
            .getBooleanTopic("Complete").publish();
    private final BooleanPublisher shooterCharacterizationAbortedPublisher = shooterCharacterizationTable
            .getBooleanTopic("Aborted").publish();
    private final IntegerPublisher shooterCharacterizationRunIdPublisher = shooterCharacterizationTable
            .getIntegerTopic("RunId").publish();
    private final IntegerPublisher shooterCharacterizationModeCodePublisher = shooterCharacterizationTable
            .getIntegerTopic("ModeCode").publish();
    private final StringPublisher shooterCharacterizationModeLabelPublisher = shooterCharacterizationTable
            .getStringTopic("ModeLabel").publish();
    private final IntegerPublisher shooterCharacterizationSegmentIndexPublisher = shooterCharacterizationTable
            .getIntegerTopic("SegmentIndex").publish();
    private final IntegerPublisher shooterCharacterizationSegmentCountPublisher = shooterCharacterizationTable
            .getIntegerTopic("SegmentCount").publish();
    private final StringPublisher shooterCharacterizationAbortReasonPublisher = shooterCharacterizationTable
            .getStringTopic("AbortReason").publish();

    private final NetworkTable hopperTable = telemetryTable.getSubTable("Hopper");
    private final StructPublisher<HopperSnapshot> hopperStatePublisher = hopperTable
            .getStructTopic("State", HopperSnapshot.struct).publish();
    private final StructPublisher<HopperPreloadSnapshot> hopperPreloadPublisher = hopperTable
            .getStructTopic("Preload", HopperPreloadSnapshot.struct).publish();
    private final BooleanPublisher hopperTopBeamBreakRawBlockedPublisher = hopperTable
            .getBooleanTopic("TopBeamBreakRawBlocked").publish();
    private final BooleanPublisher hopperTopBeamBreakBlockedPublisher = hopperTable
            .getBooleanTopic("TopBeamBreakBlocked").publish();
    private final NetworkTable hopperFeederTable = hopperTable.getSubTable("Feeder");
    private final DoublePublisher hopperFeederCommandedVoltsPublisher = TelemetryUtil.publishDouble(
            hopperFeederTable, "CommandedVolts", "V");
    private final NetworkTable hopperFeederLeftTable = hopperFeederTable.getSubTable("Left");
    private final DoublePublisher hopperFeederLeftVelocityPublisher = TelemetryUtil.publishDouble(
            hopperFeederLeftTable, "VelocityRps", "rps");
    private final DoublePublisher hopperFeederLeftSupplyCurrentPublisher = TelemetryUtil.publishDouble(
            hopperFeederLeftTable, "SupplyCurrentAmps", "A");
    private final DoublePublisher hopperFeederLeftStatorCurrentPublisher = TelemetryUtil.publishDouble(
            hopperFeederLeftTable, "StatorCurrentAmps", "A");
    private final DoublePublisher hopperFeederLeftAppliedVoltsPublisher = TelemetryUtil.publishDouble(
            hopperFeederLeftTable, "AppliedVolts", "V");
    private final NetworkTable hopperFeederRightTable = hopperFeederTable.getSubTable("Right");
    private final DoublePublisher hopperFeederRightVelocityPublisher = TelemetryUtil.publishDouble(
            hopperFeederRightTable, "VelocityRps", "rps");
    private final DoublePublisher hopperFeederRightSupplyCurrentPublisher = TelemetryUtil.publishDouble(
            hopperFeederRightTable, "SupplyCurrentAmps", "A");
    private final DoublePublisher hopperFeederRightStatorCurrentPublisher = TelemetryUtil.publishDouble(
            hopperFeederRightTable, "StatorCurrentAmps", "A");
    private final DoublePublisher hopperFeederRightAppliedVoltsPublisher = TelemetryUtil.publishDouble(
            hopperFeederRightTable, "AppliedVolts", "V");

    private final NetworkTable superstructureTable = telemetryTable.getSubTable("Superstructure");
    private final StructPublisher<SuperstructureSnapshot> superstructureStatePublisher = superstructureTable
            .getStructTopic("State", SuperstructureSnapshot.struct).publish();

    private final NetworkTable visionTable = telemetryTable.getSubTable("Vision");
    private final StructPublisher<VisionSnapshot> visionStatePublisher = visionTable
            .getStructTopic("State", VisionSnapshot.struct).publish();
    private final IntegerPublisher visionValidationStatusPublisher = visionTable.getIntegerTopic("ValidationStatusCode")
            .publish();

    private final NetworkTable simTable = telemetryTable.getSubTable("Sim");
    private final StructPublisher<SimSnapshot> simStatePublisher = simTable.getStructTopic("State", SimSnapshot.struct)
            .publish();
    private final StructArrayPublisher<Pose3d> simBallPosesPublisher = simTable.getStructArrayTopic("BallPoses", Pose3d.struct)
            .publish();
    private final StructPublisher<Pose3d> simTargetPosePublisher = simTable.getStructTopic("TargetPose", Pose3d.struct)
            .publish();

    private final NetworkTable controlsTable = telemetryTable.getSubTable("Controls");
    private final BooleanPublisher driveAutoAimActivePublisher = controlsTable.getBooleanTopic("Drive/AutoAimActive").publish();
    private final DoublePublisher driveAutoAimRotCmdPublisher = TelemetryUtil.publishDouble(controlsTable,
            "Drive/AutoAimRotCmdRadPerSec", "rad/s");
    private final DoublePublisher driveAutoAimTransLimitPublisher = TelemetryUtil.publishDouble(controlsTable,
            "Drive/AutoAimTransLimitMps", "m/s");
    private final StructPublisher<YShotSnapshot> yShotPublisher = controlsTable.getStructTopic("YShot/State", YShotSnapshot.struct)
            .publish();
    private final StructPublisher<TimedShotSnapshot> timedShotPublisher = controlsTable
            .getStructTopic("TimedShot/State", TimedShotSnapshot.struct).publish();

    private final NetworkTable practiceTimerTable = telemetryTable.getSubTable("PracticeTimer");
    private final StructPublisher<PracticeTimerSnapshot> practiceTimerStatePublisher = practiceTimerTable
            .getStructTopic("State", PracticeTimerSnapshot.struct).publish();
    private final IntegerPublisher practiceTimerPhaseCodePublisher = practiceTimerTable.getIntegerTopic("PhaseCode").publish();
    private final StringPublisher practiceTimerPhaseLabelPublisher = practiceTimerTable.getStringTopic("PhaseLabel").publish();
    private final BooleanPublisher practiceTimerActivePublisher = practiceTimerTable.getBooleanTopic("Active").publish();
    private final BooleanPublisher practiceTimerCompletePublisher = practiceTimerTable.getBooleanTopic("Complete").publish();
    private final DoublePublisher practiceTimerPhaseRemainingPublisher = TelemetryUtil.publishDouble(
            practiceTimerTable, "PhaseRemainingSec", "s");
    private final DoublePublisher practiceTimerTotalRemainingPublisher = TelemetryUtil.publishDouble(
            practiceTimerTable, "TotalRemainingSec", "s");
    private final IntegerPublisher practiceTimerRunIdPublisher = practiceTimerTable.getIntegerTopic("RunId").publish();
    private final BooleanPublisher practiceTimerUsingDsMatchTimePublisher = practiceTimerTable
            .getBooleanTopic("UsingDsMatchTime").publish();
    private final IntegerPublisher practiceTimerZoneCodePublisher = practiceTimerTable.getIntegerTopic("ZoneCode").publish();
    private final StringPublisher practiceTimerZoneLabelPublisher = practiceTimerTable.getStringTopic("ZoneLabel").publish();
    private final BooleanPublisher practiceTimerZoneActivePublisher = practiceTimerTable.getBooleanTopic("ZoneActive").publish();
    private final BooleanPublisher practiceTimerZoneKnownPublisher = practiceTimerTable.getBooleanTopic("ZoneKnown").publish();
    private final BooleanPublisher practiceTimerEndgamePublisher = practiceTimerTable.getBooleanTopic("Endgame").publish();
    private final DoublePublisher practiceTimerZoneRemainingPublisher = TelemetryUtil.publishDouble(
            practiceTimerTable, "ZoneRemainingSec", "s");
    private final DoublePublisher practiceTimerTeleopRemainingPublisher = TelemetryUtil.publishDouble(
            practiceTimerTable, "TeleopRemainingSec", "s");

    private final Map<String, VisionCameraPublishers> visionCameraPublishers = new HashMap<>();

    private final Map<String, StructPublisher<Translation2d>> translationPublishers = new HashMap<>();
    private final Map<String, StructPublisher<Rotation2d>> rotationPublishers = new HashMap<>();
    private final Map<String, StructArrayPublisher<Pose2d>> pose2dArrayPublishers = new HashMap<>();
    private final Map<String, StructArrayPublisher<Pose3d>> pose3dArrayPublishers = new HashMap<>();

    private int lastVisionValidationStatusCode = VALIDATION_OK;

    private TelemetryManager() {
    }

    public void publishDrivePose(Pose2d pose) {
        drivePosePublisher.set(pose);
    }

    public void publishDriveSpeeds(ChassisSpeeds speeds) {
        driveSpeedsPublisher.set(speeds);
    }

    public void publishDriveModuleStates(SwerveModuleState[] states) {
        driveModuleStatesPublisher.set(states);
    }

    public void publishDriveModuleTargets(SwerveModuleState[] targets) {
        driveModuleTargetsPublisher.set(targets);
    }

    public void publishDriveModulePositions(SwerveModulePosition[] positions) {
        driveModulePositionsPublisher.set(positions);
    }

    public void publishDriveModuleElectrical(double[] driveVelocityRps, double[] driveStatorCurrentAmps) {
        driveModuleVelocityPublisher.set(driveVelocityRps);
        driveModuleStatorCurrentPublisher.set(driveStatorCurrentAmps);
    }

    public void publishDriveStatus(DriveStatusSnapshot snapshot) {
        driveStatusPublisher.set(snapshot);
    }

    public void publishDriveAutoPathActive(boolean active) {
        driveAutoActivePublisher.set(active);
    }

    public void publishDriveAutoPathSample(
            double sampleTimestampSec,
            Pose2d desiredPose,
            Pose2d measuredPose,
            Translation2d poseError,
            double headingErrorRad,
            ChassisSpeeds feedforwardSpeeds,
            ChassisSpeeds feedbackSpeeds,
            ChassisSpeeds commandedSpeeds,
            ChassisSpeeds measuredSpeeds,
            double[] moduleForcesX,
            double[] moduleForcesY) {
        driveAutoActivePublisher.set(true);
        driveAutoLastSampleTimestampPublisher.set(sampleTimestampSec);
        driveAutoDesiredPosePublisher.set(desiredPose);
        driveAutoMeasuredPosePublisher.set(measuredPose);
        driveAutoPoseErrorPublisher.set(poseError);
        driveAutoPoseErrorNormPublisher.set(poseError.getNorm());
        driveAutoHeadingErrorPublisher.set(headingErrorRad);
        driveAutoFeedforwardSpeedsPublisher.set(feedforwardSpeeds);
        driveAutoFeedbackSpeedsPublisher.set(feedbackSpeeds);
        driveAutoCommandedSpeedsPublisher.set(commandedSpeeds);
        driveAutoMeasuredSpeedsPublisher.set(measuredSpeeds);
        driveAutoModuleForcesXPublisher.set(moduleForcesX);
        driveAutoModuleForcesYPublisher.set(moduleForcesY);
    }

    public void publishIntakeState(IntakeSnapshot snapshot) {
        intakeStatePublisher.set(snapshot);
        intakePivotStatorCurrentPublisher.set(snapshot.pivotStatorCurrentAmps());
        intakeLeftRollerStatorCurrentPublisher.set(snapshot.leftRollerStatorCurrentAmps());
        intakeRightRollerStatorCurrentPublisher.set(snapshot.rightRollerStatorCurrentAmps());
    }

    public void publishShooterState(ShooterSnapshot snapshot) {
        shooterStatePublisher.set(snapshot);
    }

    public void publishAutoAimSolver(AutoAimSolverSnapshot snapshot) {
        shooterSolverPublisher.set(snapshot);
    }

    public void publishShooterCharacterization(ShooterCharacterizationSnapshot snapshot) {
        shooterCharacterizationPublisher.set(snapshot);
    }

    public void publishShooterCharacterizationStatus(
            boolean active,
            boolean complete,
            boolean aborted,
            int runId,
            int modeCode,
            String modeLabel,
            int segmentIndex,
            int segmentCount,
            String abortReason) {
        shooterCharacterizationActivePublisher.set(active);
        shooterCharacterizationCompletePublisher.set(complete);
        shooterCharacterizationAbortedPublisher.set(aborted);
        shooterCharacterizationRunIdPublisher.set(runId);
        shooterCharacterizationModeCodePublisher.set(modeCode);
        shooterCharacterizationModeLabelPublisher.set(modeLabel);
        shooterCharacterizationSegmentIndexPublisher.set(segmentIndex);
        shooterCharacterizationSegmentCountPublisher.set(segmentCount);
        shooterCharacterizationAbortReasonPublisher.set(abortReason);
    }

    public void publishHopperState(HopperSnapshot snapshot) {
        hopperStatePublisher.set(snapshot);
        hopperTopBeamBreakRawBlockedPublisher.set(snapshot.topBeamBreakRawBlocked());
        hopperTopBeamBreakBlockedPublisher.set(snapshot.topBeamBreakBlocked());
    }

    public void publishHopperPreloadState(HopperPreloadSnapshot snapshot) {
        hopperPreloadPublisher.set(snapshot);
    }

    public void publishHopperFeederState(
            double commandedVolts,
            double leftVelocityRps,
            double leftSupplyCurrentAmps,
            double leftStatorCurrentAmps,
            double leftAppliedVolts,
            double rightVelocityRps,
            double rightSupplyCurrentAmps,
            double rightStatorCurrentAmps,
            double rightAppliedVolts) {
        hopperFeederCommandedVoltsPublisher.set(commandedVolts);
        hopperFeederLeftVelocityPublisher.set(leftVelocityRps);
        hopperFeederLeftSupplyCurrentPublisher.set(leftSupplyCurrentAmps);
        hopperFeederLeftStatorCurrentPublisher.set(leftStatorCurrentAmps);
        hopperFeederLeftAppliedVoltsPublisher.set(leftAppliedVolts);
        hopperFeederRightVelocityPublisher.set(rightVelocityRps);
        hopperFeederRightSupplyCurrentPublisher.set(rightSupplyCurrentAmps);
        hopperFeederRightStatorCurrentPublisher.set(rightStatorCurrentAmps);
        hopperFeederRightAppliedVoltsPublisher.set(rightAppliedVolts);
    }

    public void publishSuperstructureState(SuperstructureSnapshot snapshot) {
        superstructureStatePublisher.set(snapshot);
    }

    public void publishVisionState(VisionSnapshot snapshot) {
        visionStatePublisher.set(snapshot);
    }

    public void publishVisionValidationStatusCode(int statusCode) {
        lastVisionValidationStatusCode = statusCode;
        visionValidationStatusPublisher.set(statusCode);
    }

    public int getVisionValidationStatusCode() {
        return lastVisionValidationStatusCode;
    }

    public void publishVisionCameraState(String cameraName, VisionCameraSnapshot snapshot) {
        getVisionCameraPublishers(cameraName).statePublisher.set(snapshot);
    }

    public void publishVisionCameraGeometry(
            String cameraName,
            Pose3d[] tagPoses,
            Pose3d cameraPose,
            Pose3d robotPose,
            Pose2d relevantPoseEstimate) {
        VisionCameraPublishers publishers = getVisionCameraPublishers(cameraName);
        publishers.tagPosesPublisher.set(tagPoses);
        if (cameraPose != null) {
            publishers.cameraPosePublisher.set(cameraPose);
        }
        if (robotPose != null) {
            publishers.robotPosePublisher.set(robotPose);
        }
        if (relevantPoseEstimate != null) {
            publishers.relevantPosePublisher.set(relevantPoseEstimate);
        }
    }

    public void publishSimState(SimSnapshot snapshot) {
        simStatePublisher.set(snapshot);
    }

    public void publishSimGeometry(Pose3d[] ballPoses, Pose3d targetPose) {
        simBallPosesPublisher.set(ballPoses);
        simTargetPosePublisher.set(targetPose);
    }

    public void publishDriveAutoAim(boolean active, double rotCmdRadPerSec, double transLimitMps) {
        driveAutoAimActivePublisher.set(active);
        driveAutoAimRotCmdPublisher.set(rotCmdRadPerSec);
        driveAutoAimTransLimitPublisher.set(transLimitMps);
    }

    public void publishYShotState(YShotSnapshot snapshot) {
        yShotPublisher.set(snapshot);
    }

    public void publishTimedShotState(TimedShotSnapshot snapshot) {
        timedShotPublisher.set(snapshot);
    }

    public void publishPracticeTimerState(PracticeTimerSnapshot snapshot, String phaseLabel, String zoneLabel) {
        practiceTimerStatePublisher.set(snapshot);
        practiceTimerPhaseCodePublisher.set(snapshot.phaseCode());
        practiceTimerPhaseLabelPublisher.set(phaseLabel);
        practiceTimerActivePublisher.set(snapshot.active());
        practiceTimerCompletePublisher.set(snapshot.complete());
        practiceTimerPhaseRemainingPublisher.set(snapshot.phaseRemainingSec());
        practiceTimerTotalRemainingPublisher.set(snapshot.totalRemainingSec());
        practiceTimerRunIdPublisher.set(snapshot.runId());
        practiceTimerUsingDsMatchTimePublisher.set(snapshot.usingDsMatchTime());
        practiceTimerZoneCodePublisher.set(snapshot.zoneCode());
        practiceTimerZoneLabelPublisher.set(zoneLabel);
        practiceTimerZoneActivePublisher.set(snapshot.zoneActive());
        practiceTimerZoneKnownPublisher.set(snapshot.zoneKnown());
        practiceTimerEndgamePublisher.set(snapshot.endgame());
        practiceTimerZoneRemainingPublisher.set(snapshot.zoneRemainingSec());
        practiceTimerTeleopRemainingPublisher.set(snapshot.teleopRemainingSec());
    }

    public void publishTranslation2d(String key, Translation2d translation) {
        StructPublisher<Translation2d> publisher = translationPublishers.computeIfAbsent(key,
                k -> telemetryTable.getStructTopic("Logs/" + k, Translation2d.struct).publish());
        publisher.set(translation);
    }

    public void publishRotation2d(String key, Rotation2d rotation) {
        StructPublisher<Rotation2d> publisher = rotationPublishers.computeIfAbsent(key,
                k -> telemetryTable.getStructTopic("Logs/" + k, Rotation2d.struct).publish());
        publisher.set(rotation);
    }

    public void publishPose2dArray(String key, Pose2d[] poses) {
        StructArrayPublisher<Pose2d> publisher = pose2dArrayPublishers.computeIfAbsent(key,
                k -> telemetryTable.getStructArrayTopic("Logs/" + k, Pose2d.struct).publish());
        publisher.set(poses);
    }

    public void publishPose3dArray(String key, Pose3d[] poses) {
        StructArrayPublisher<Pose3d> publisher = pose3dArrayPublishers.computeIfAbsent(key,
                k -> telemetryTable.getStructArrayTopic("Logs/" + k, Pose3d.struct).publish());
        publisher.set(poses);
    }

    private VisionCameraPublishers getVisionCameraPublishers(String cameraName) {
        return visionCameraPublishers.computeIfAbsent(cameraName, this::createVisionCameraPublishers);
    }

    private VisionCameraPublishers createVisionCameraPublishers(String cameraName) {
        NetworkTable cameraTable = visionTable.getSubTable(cameraName);
        return new VisionCameraPublishers(
                cameraTable.getStructTopic("State", VisionCameraSnapshot.struct).publish(),
                cameraTable.getStructArrayTopic("TagPoses", Pose3d.struct).publish(),
                cameraTable.getStructTopic("CameraPose", Pose3d.struct).publish(),
                cameraTable.getStructTopic("RobotPose", Pose3d.struct).publish(),
                cameraTable.getStructTopic("RelevantPoseEstimate", Pose2d.struct).publish());
    }

    private record VisionCameraPublishers(
            StructPublisher<VisionCameraSnapshot> statePublisher,
            StructArrayPublisher<Pose3d> tagPosesPublisher,
            StructPublisher<Pose3d> cameraPosePublisher,
            StructPublisher<Pose3d> robotPosePublisher,
            StructPublisher<Pose2d> relevantPosePublisher) {
    }
}
