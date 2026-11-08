package com.team9470.simulation;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

import static edu.wpi.first.units.Units.*;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;
import java.util.function.Supplier;

import com.team9470.FieldConstants;
import com.team9470.subsystems.intake.Intake;
import com.team9470.subsystems.shooter.ShooterConstants;
import com.team9470.subsystems.swerve.Swerve;
import com.team9470.telemetry.TelemetryManager;
import com.team9470.telemetry.structs.SimSnapshot;
import com.team9470.util.AutoAim;

/**
 * Manages all projectile simulation - spawning, physics, visualization.
 * Separate from Shooter subsystem to keep sim logic out of robot code.
 */
public class ProjectileSimulation {

    private static ProjectileSimulation instance;

    public static ProjectileSimulation getInstance() {
        if (instance == null) {
            instance = new ProjectileSimulation();
        }
        return instance;
    }

    // Projectile list
    private final List<PhysicsSim.Projectile> projectiles = new ArrayList<>();

    // Firing state
    private final List<Double> shotTimestamps = new ArrayList<>();
    private double lastLaunchTime = 0.0;
    private double measuredBPS = 0.0;
    private boolean shootLeft = true;
    private final TelemetryManager telemetry = TelemetryManager.getInstance();

    // Flywheel/Hood Simulation
    private final FlywheelSim flywheelSim;
    private final SingleJointedArmSim hoodSim;

    // Context suppliers
    private Supplier<Pose2d> poseSupplier = Pose2d::new;
    private Supplier<ChassisSpeeds> speedsSupplier = ChassisSpeeds::new;
    private Supplier<Double> flywheelVoltageSupplier = () -> 0.0;
    private Supplier<Double> hoodVoltageSupplier = () -> 0.0;

    private ProjectileSimulation() {
        // Flywheel simulation
        flywheelSim = new FlywheelSim(
                LinearSystemId.createFlywheelSystem(
                        DCMotor.getKrakenX60(4),
                        ShooterConstants.kFlywheelMOI.in(KilogramSquareMeters),
                        ShooterConstants.kFlywheelGearRatio),
                DCMotor.getKrakenX60(4),
                ShooterConstants.kFlywheelGearRatio);

        // Hood simulation
        hoodSim = new SingleJointedArmSim(
                DCMotor.getKrakenX60(1),
                ShooterConstants.kHoodGearRatio,
                ShooterConstants.kHoodMOI.in(KilogramSquareMeters),
                ShooterConstants.kHoodLength.in(Meters),
                ShooterConstants.kMinHoodAngle.in(Radians),
                ShooterConstants.kMaxHoodAngle.in(Radians),
                true, // Simulate Gravity
                0.0);
    }

    /**
     * Connect to robot state suppliers.
     */
    public void setContext(
            Supplier<Pose2d> pose,
            Supplier<ChassisSpeeds> speeds,
            Supplier<Double> flywheelVoltage,
            Supplier<Double> hoodVoltage) {
        this.poseSupplier = pose;
        this.speedsSupplier = speeds;
        this.flywheelVoltageSupplier = flywheelVoltage;
        this.hoodVoltageSupplier = hoodVoltage;
    }

    /**
     * Main simulation update - call from Robot.simulationPeriodic().
     */
    public void update(double dt, boolean isFiring, double targetSpeedRPS, double targetHoodAngleRot) {
        double now = Timer.getFPGATimestamp();

        // Update mechanism physics
        flywheelSim.setInputVoltage(flywheelVoltageSupplier.get());
        hoodSim.setInputVoltage(hoodVoltageSupplier.get());
        flywheelSim.update(dt);
        hoodSim.update(dt);

        // BPS tracking
        if (isFiring) {
            shotTimestamps.removeIf(t -> (now - t) > 1.0);
            measuredBPS = shotTimestamps.size();
        } else {
            shotTimestamps.clear();
            measuredBPS = 0.0;
        }

        // Auto-fire when at setpoint
        if (isFiring && isFlywheelAtSetpoint(targetSpeedRPS) && isHoodAtSetpoint(targetHoodAngleRot)) {
            spawnProjectile(targetSpeedRPS, targetHoodAngleRot);
            shotTimestamps.add(now);
            lastLaunchTime = now;
        }

        // Update projectile physics
        updateProjectiles(dt);

        // Publish visualization
        publish();

        telemetry.publishSimState(new SimSnapshot(
                flywheelSim.getAngularVelocityRPM() / 60.0,
                hoodSim.getAngleRads(),
                projectiles.size(),
                measuredBPS));
    }

    private boolean isFlywheelAtSetpoint(double targetRPS) {
        double currentRPS = flywheelSim.getAngularVelocityRPM() / 60.0;
        return Math.abs(currentRPS - targetRPS) < 0.5;
    }

    private boolean isHoodAtSetpoint(double targetHoodRot) {
        // Sim uses launch angle; convert to mechanism rotations for comparison.
        double launchRad = hoodSim.getAngleRads();
        double currentHoodRot = ShooterConstants.launchRadToMechanismRotations(launchRad);
        return Math.abs(currentHoodRot - targetHoodRot) < 0.01;
    }

    private void spawnProjectile(double targetSpeedRPS, double targetHoodAngleRot) {
        Pose2d robotPose = poseSupplier.get();
        if (robotPose == null)
            return;

        PhysicsSim.Projectile p = new PhysicsSim.Projectile(new Translation3d(), new Translation3d());
        projectiles.add(p);

        // Get actual sim state
        double flywheelRPS = flywheelSim.getAngularVelocityRPM() / 60.0;
        double hoodRad = hoodSim.getAngleRads();

        // Calculate exit velocity
        double wheelRadius = Units.inchesToMeters(2.0);
        double efficiency = ShooterConstants.kFlywheelEfficiency;
        double surfaceSpeed = flywheelRPS * 2.0 * Math.PI * wheelRadius;
        double speed = surfaceSpeed * efficiency;

        // Apply recoil to flywheel
        double ballMass = 0.27; // kg
        double ballKE = 0.5 * ballMass * speed * speed;
        double moi = ShooterConstants.kFlywheelMOI.in(KilogramSquareMeters);
        double flywheelRadPerSec = Units.rotationsToRadians(flywheelRPS);
        double flywheelKE = 0.5 * moi * flywheelRadPerSec * flywheelRadPerSec;
        double newFlywheelKE = Math.max(0, flywheelKE - ballKE);
        double newFlywheelRadPerSec = Math.sqrt(2 * newFlywheelKE / moi);
        flywheelSim.setState(VecBuilder.fill(newFlywheelRadPerSec));

        // Muzzle position
        Translation3d muzzleOffset = new Translation3d(
                ShooterConstants.kShooterOffsetX.in(Meters),
                shootLeft ? 0.1 : -0.1,
                ShooterConstants.kShooterOffsetZ.in(Meters));
        shootLeft = !shootLeft;

        Translation3d muzzleWorld = new Translation3d(robotPose.getX(), robotPose.getY(), 0)
                .plus(muzzleOffset.rotateBy(new Rotation3d(0, 0, robotPose.getRotation().getRadians())));

        // Launch direction - sim angle is launch angle (from horizontal)
        Rotation2d yawField = robotPose.getRotation();
        double launchPitch = hoodRad; // hoodSim returns launch angle

        double cosP = Math.cos(launchPitch);
        double sinP = Math.sin(launchPitch);
        double cosY = yawField.getCos();
        double sinY = yawField.getSin();

        Translation3d launchDir = new Translation3d(cosP * cosY, cosP * sinY, sinP);
        Translation3d initialVel = launchDir.times(speed);

        // Add robot velocity
        ChassisSpeeds speeds = speedsSupplier.get();
        initialVel = initialVel.plus(new Translation3d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, 0));

        // Add spin (magnus effect)
        double spinRPM = flywheelRPS * 60.0 * 0.5;
        Translation3d angularVel = new Translation3d(0, Units.rotationsToRadians(spinRPM / 60.0), 0);

        p.position = muzzleWorld;
        p.velocity = initialVel;
        p.angularVelocity = angularVel;
    }

    private void updateProjectiles(double dt) {
        Pose2d robotPose = Swerve.getInstance().getPose();
        ChassisSpeeds robotSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(
                Swerve.getInstance().getChassisSpeeds(),
                robotPose.getRotation());

        var intake = Intake.getInstance();
        double intakeAngle = intake.getPivotAngle();
        boolean intakeActive = intake.isRunning();

        PhysicsSim.simulate(dt, projectiles, AutoAim.getTarget(robotPose), robotPose, robotSpeeds,
                intakeAngle, intakeActive);

        // Remove scored projectiles
        Iterator<PhysicsSim.Projectile> iter = projectiles.iterator();
        while (iter.hasNext()) {
            PhysicsSim.Projectile p = iter.next();
            if (p.isScored) {
                iter.remove();
            }
        }
    }

    private void publish() {
        Pose3d[] ballPoses = projectiles.stream()
                .map(p -> new Pose3d(p.position, new Rotation3d()))
                .toArray(Pose3d[]::new);
        Pose2d pose = poseSupplier.get();
        if (pose == null) {
            pose = new Pose2d();
        }
        telemetry.publishSimGeometry(ballPoses, new Pose3d(AutoAim.getTarget(pose), new Rotation3d()));
    }

    // Accessors for Shooter to update motor sim states
    public double getFlywheelVelocityRPS() {
        return flywheelSim.getAngularVelocityRPM() / 60.0;
    }

    public double getHoodAngleRad() {
        return hoodSim.getAngleRads();
    }

    public double getHoodVelocityRadPerSec() {
        return hoodSim.getVelocityRadPerSec();
    }

    public double getMeasuredBPS() {
        return measuredBPS;
    }

    public List<PhysicsSim.Projectile> getProjectiles() {
        return projectiles;
    }

    /**
     * Debug: Seed field with game pieces.
     */
    public void seedField() {
        double fieldCenterX = FieldConstants.fieldLength / 2.0;
        double fieldCenterY = FieldConstants.fieldWidth / 2.0;

        double fuelDepth = Units.inchesToMeters(72.0);
        double fuelWidth = Units.inchesToMeters(206.0);
        double gapWidth = Units.inchesToMeters(9.0);
        double spacing = 0.16;

        double minX = fieldCenterX - (fuelDepth / 2.0);
        double maxX = fieldCenterX + (fuelDepth / 2.0);
        double minY = fieldCenterY - (fuelWidth / 2.0);
        double maxY = fieldCenterY + (fuelWidth / 2.0);

        for (double x = minX; x < maxX; x += spacing) {
            for (double y = minY; y < maxY; y += spacing) {
                if (Math.abs(y - fieldCenterY) < (gapWidth / 2.0)) {
                    continue;
                }
                projectiles.add(new PhysicsSim.Projectile(
                        new Translation3d(x, y, 0.1),
                        new Translation3d()));
            }
        }
    }
}
