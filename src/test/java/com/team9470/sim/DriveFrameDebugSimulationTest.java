package com.team9470.sim;

import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.team9470.subsystems.swerve.Swerve;
import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import org.junit.jupiter.api.Test;

class DriveFrameDebugSimulationTest {
    @Test
    void printResetHeadingBehavior() throws Exception {
        HAL.initialize(500, 0);
        DriverStationSim.resetData();
        DriverStationSim.setDsAttached(true);
        DriverStationSim.setAllianceStationId(AllianceStationID.Blue1);
        DriverStationSim.setEnabled(true);
        DriverStationSim.setAutonomous(true);
        DriverStationSim.notifyNewData();

        Swerve swerve = Swerve.getInstance();
        runCase(swerve, Pose2d.kZero, "zero");
        runCase(swerve, new Pose2d(4.4, 7.5, Rotation2d.kCCW_Pi_2), "choreo-start");
    }

    private static void runCase(Swerve swerve, Pose2d resetPose, String label) throws Exception {
        swerve.resetPose(resetPose);
        Thread.sleep(100);
        var initial = swerve.getStateCopy();
        System.out.printf("%s initial pose=(%.3f, %.3f, %.1fdeg) raw=%.1fdeg%n",
                label,
                initial.Pose.getX(),
                initial.Pose.getY(),
                initial.Pose.getRotation().getDegrees(),
                initial.RawHeading.getDegrees());

        var request = new SwerveRequest.ApplyFieldSpeeds()
                .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage)
                .withSpeeds(new edu.wpi.first.math.kinematics.ChassisSpeeds(2.0, 0.0, 0.0));
        for (int i = 0; i < 120; i++) {
            DriverStation.refreshData();
            swerve.setControl(request);
            Thread.sleep(20);
        }

        var state = swerve.getStateCopy();
        System.out.printf("%s final pose=(%.3f, %.3f, %.1fdeg) raw=%.1fdeg speeds=(%.3f, %.3f, %.3f)%n",
                label,
                state.Pose.getX(),
                state.Pose.getY(),
                state.Pose.getRotation().getDegrees(),
                state.RawHeading.getDegrees(),
                state.Speeds.vxMetersPerSecond,
                state.Speeds.vyMetersPerSecond,
                state.Speeds.omegaRadiansPerSecond);
        for (int i = 0; i < state.ModuleStates.length; i++) {
            System.out.printf("%s module %d state=(%.3f, %.1fdeg) target=(%.3f, %.1fdeg)%n",
                    label,
                    i,
                    state.ModuleStates[i].speedMetersPerSecond,
                    state.ModuleStates[i].angle.getDegrees(),
                    state.ModuleTargets[i].speedMetersPerSecond,
                    state.ModuleTargets[i].angle.getDegrees());
        }
    }
}
