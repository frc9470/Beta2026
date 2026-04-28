package com.team9470.sim;

import choreo.Choreo;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import com.team9470.subsystems.swerve.Swerve;
import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import org.junit.jupiter.api.Test;

class ChoreoFollowDebugSimulationTest {
    @Test
    void printFirstPathTracking() throws Exception {
        HAL.initialize(500, 0);
        DriverStationSim.resetData();
        DriverStationSim.setDsAttached(true);
        DriverStationSim.setAllianceStationId(AllianceStationID.Blue1);
        DriverStationSim.setEnabled(true);
        DriverStationSim.setAutonomous(true);
        DriverStationSim.notifyNewData();

        runPath("bumpCycle1Fast");
    }

    private static void runPath(String name) throws Exception {
        Swerve swerve = Swerve.getInstance();
        Trajectory<SwerveSample> trajectory = Choreo.<SwerveSample>loadTrajectory(name).orElseThrow();
        Pose2d initialPose = trajectory.getInitialPose(false).orElseThrow();
        swerve.resetPose(initialPose);
        Thread.sleep(100);

        double start = Timer.getTimestamp();
        double lastPrint = -1.0;
        double maxTime = trajectory.getFinalSample(false).orElseThrow().t + 0.25;
        while (Timer.getTimestamp() - start <= maxTime) {
            DriverStationSim.notifyNewData();
            double t = Timer.getTimestamp() - start;
            SwerveSample sample = trajectory.sampleAt(t, false).orElseThrow();
            swerve.followPath(sample);
            var state = swerve.getStateCopy();
            if (t - lastPrint >= 0.25 || (lastPrint < 1.0 && t >= 1.05)) {
                lastPrint = t;
                System.out.printf(
                        "t=%.3f desired=(%.3f,%.3f,%.1f) actual=(%.3f,%.3f,%.1f) speeds=(%.2f,%.2f,%.2f)%n",
                        t,
                        sample.x,
                        sample.y,
                        Math.toDegrees(sample.heading),
                        state.Pose.getX(),
                        state.Pose.getY(),
                        state.Pose.getRotation().getDegrees(),
                        state.Speeds.vxMetersPerSecond,
                        state.Speeds.vyMetersPerSecond,
                        state.Speeds.omegaRadiansPerSecond);
            }
            Thread.sleep(20);
        }

        var state = swerve.getStateCopy();
        System.out.printf("final actual=(%.3f,%.3f,%.1f)%n",
                state.Pose.getX(),
                state.Pose.getY(),
                state.Pose.getRotation().getDegrees());
    }
}
