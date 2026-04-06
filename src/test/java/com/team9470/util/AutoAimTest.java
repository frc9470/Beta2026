package com.team9470.util;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.team9470.FieldConstants;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import org.junit.jupiter.api.Test;

class AutoAimTest {
    private static Pose2d hubAlignedPose() {
        return new Pose2d(
                FieldConstants.Hub.topCenterPoint.getX() - 2.5,
                FieldConstants.Hub.topCenterPoint.getY(),
                new Rotation2d());
    }

    private static Pose2d feedPose(double yMeters) {
        return new Pose2d(6.0, yMeters, new Rotation2d());
    }

    @Test
    void stationaryRobotProducesZeroTargetOmega() {
        var solution = AutoAim.calculate(hubAlignedPose(), new ChassisSpeeds());
        assertTrue(solution.isValid());
        assertEquals(0.0, solution.targetOmega(), 1e-9);
    }

    @Test
    void nonSotmStaysStaticUnderStrafe() {
        Pose2d pose = hubAlignedPose();
        var stationary = AutoAim.calculateNonSotm(pose);
        var moving = AutoAim.calculateNonSotm(pose);

        assertTrue(stationary.isValid());
        assertTrue(moving.isValid());
        assertEquals(stationary.targetRobotYaw().getRadians(), moving.targetRobotYaw().getRadians(), 1e-9);
        assertEquals(0.0, moving.targetOmega(), 1e-9);
    }

    @Test
    void sotmStrafingLeftShiftsYawClockwiseAndOmegaNegative() {
        Pose2d pose = hubAlignedPose();
        var stationary = AutoAim.calculateSotm(pose, new ChassisSpeeds());
        var moving = AutoAim.calculateSotm(pose, new ChassisSpeeds(0.0, 1.0, 0.0));

        assertTrue(moving.isValid());
        assertTrue(
                moving.targetRobotYaw().getRadians() < stationary.targetRobotYaw().getRadians() - 1e-4,
                "Lookahead should bias yaw clockwise when the robot strafes left");
        assertTrue(moving.targetOmega() < 0.0, "Angular feedforward should also command clockwise rotation");
    }

    @Test
    void defaultFeedTargetStaysLeftSide() {
        var leftPoseTarget = AutoAim.getTarget(feedPose(FieldConstants.fieldWidth - 1.0));
        var rightPoseTarget = AutoAim.getTarget(feedPose(1.0));

        assertEquals(leftPoseTarget.getY(), rightPoseTarget.getY(), 1e-9);
        assertTrue(leftPoseTarget.getY() > FieldConstants.LinesHorizontal.center);
    }

    @Test
    void dynamicFeedTargetFollowsRobotSide() {
        var leftPoseTarget = AutoAim.getTarget(feedPose(FieldConstants.fieldWidth - 1.0), true);
        var rightPoseTarget = AutoAim.getTarget(feedPose(1.0), true);

        assertTrue(leftPoseTarget.getY() > FieldConstants.LinesHorizontal.center);
        assertTrue(rightPoseTarget.getY() < FieldConstants.LinesHorizontal.center);
    }
}
