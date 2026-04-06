// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team9470;

import choreo.auto.AutoChooser;
import com.team9470.commands.Autos;
import com.team9470.commands.WheelRadiusCharacterization;
import com.team9470.telemetry.MatchTimingService;
import com.team9470.telemetry.TelemetryManager;
import com.team9470.telemetry.structs.YShotSnapshot;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.team9470.subsystems.swerve.Swerve;
import com.team9470.subsystems.Superstructure;
import com.team9470.subsystems.shooter.characterization.ShooterCharacterizationConfig;
import com.team9470.subsystems.shooter.characterization.ShooterCharacterizationMode;
import com.team9470.subsystems.shooter.ShooterConstants;
import com.team9470.subsystems.shooter.ShooterInterpolationMaps;
import com.team9470.subsystems.shooter.ShotParameter;
import com.team9470.util.AutoAim;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import com.team9470.Constants.OperatorConstants;
import com.team9470.subsystems.vision.Vision;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.function.DoubleSupplier;
import static edu.wpi.first.units.Units.*;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class RobotContainer {

  private final double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
  private final double MaxAngularRate = Math.toRadians(TunerConstants.maxAngularVelocity);

  // Swerve requests
  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1)
      .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage);

  // Subsystems
  private final Swerve m_swerve = Swerve.getInstance();
  private final Superstructure m_superstructure = Superstructure.getInstance();
  private final Vision m_vision = Vision.getInstance();
  private final Autos m_autos = new Autos(m_swerve);
  private final AutoChooser m_autoChooser = new AutoChooser();
  private final MatchTimingService matchTimingService = MatchTimingService.getInstance();
  private final TelemetryManager telemetry = TelemetryManager.getInstance();

  // Controllers
  private final CommandXboxController m_driverController = new CommandXboxController(
      OperatorConstants.kDriverControllerPort);

  // Debug Y-shot dashboard tuning (only used by Y button command)
  private static final String kDebugYShotRpmKey = "Debug/YShot/RPM";
  private static final String kDebugYShotHoodDegKey = "Debug/YShot/HoodAngleDeg";
  private static final String kActiveWindowEarlyRumbleLeadSecKey = "Debug/Rumble/ActiveLeadEarlySec";
  private static final String kActiveWindowRumbleLeadSecKey = "Debug/Rumble/ActiveLeadSec";
  private static final ShotParameter kDebugYShotDefaultShot = ShooterInterpolationMaps.getHub(2.0)
      .orElse(new ShotParameter(21.0, 1950.0));
  private static final double kDebugYShotDefaultRpm = kDebugYShotDefaultShot.flywheelRpm();
  private static final double kDebugYShotDefaultHoodDeg = kDebugYShotDefaultShot.hoodCommandDeg();
  private static final double kShootTranslationDeadband = 0.10;
  private static final double kIntakeHeadingLockMinSpeedMps = 0.15;
  private static final double kIntakeHeadingLockKp = 3.5;
  private static final double kIntakeHeadingManualOverrideDeadband = 0.10;
  private static final double kActiveWindowEarlyRumbleLeadSecDefault = 10.0;
  private static final double kActiveWindowRumbleLeadSecDefault = 5.0;
  private static final double kActiveWindowEarlyRumbleIntensity = 0.70;
  private static final double kActiveWindowRumbleIntensity = 1.00;
  private static final double kActiveWindowRumbleSec = 0.20;
  private final ShooterCharacterizationConfig shooterCharacterizationConfig = ShooterCharacterizationConfig.defaults();

  public RobotContainer() {
    // Connect swerve context to superstructure
    m_superstructure.setDriveContext(
        m_swerve::getPose,
        m_swerve::getChassisSpeeds);

    initDebugYShotDashboard();
    configureBindings();
    configureAutonomous();
  }

  private void initDebugYShotDashboard() {
    SmartDashboard.putNumber(kDebugYShotRpmKey, kDebugYShotDefaultRpm);
    SmartDashboard.putNumber(kDebugYShotHoodDegKey, kDebugYShotDefaultHoodDeg);
    SmartDashboard.putNumber(kActiveWindowEarlyRumbleLeadSecKey, kActiveWindowEarlyRumbleLeadSecDefault);
    SmartDashboard.putNumber(kActiveWindowRumbleLeadSecKey, kActiveWindowRumbleLeadSecDefault);
  }

  private final SwerveRequest.SwerveDriveBrake xLock = new SwerveRequest.SwerveDriveBrake();

  private double getDefaultDriveRotationRate(double vX, double vY, double rightXInput) {
    double manualRotationRate = -rightXInput * MaxAngularRate;

    // Heading lock is only active while left trigger is held.
    if (!m_driverController.leftTrigger().getAsBoolean()) {
      return manualRotationRate;
    }
    if (Math.abs(rightXInput) > kIntakeHeadingManualOverrideDeadband) {
      return manualRotationRate;
    }

    double speed = Math.hypot(vX, vY);
    if (speed < kIntakeHeadingLockMinSpeedMps) {
      return 0.0;
    }

    Rotation2d desiredHeading = new Rotation2d(vX, vY);
    double headingErrorRad = MathUtil.angleModulus(
        desiredHeading.minus(m_swerve.getPose().getRotation()).getRadians());
    return MathUtil.clamp(headingErrorRad * kIntakeHeadingLockKp, -MaxAngularRate, MaxAngularRate);
  }

  private double getShootTranslationSpeed(double rawAxisInput) {
    return -MathUtil.applyDeadband(rawAxisInput, kShootTranslationDeadband) * MaxSpeed;
  }

  private double getActiveWindowEarlyRumbleLeadSec() {
    return Math.max(0.0,
        SmartDashboard.getNumber(kActiveWindowEarlyRumbleLeadSecKey, kActiveWindowEarlyRumbleLeadSecDefault));
  }

  private double getActiveWindowRumbleLeadSec() {
    return Math.max(0.0,
        SmartDashboard.getNumber(kActiveWindowRumbleLeadSecKey, kActiveWindowRumbleLeadSecDefault));
  }

  private void setDriverRumble(double intensity) {
    m_driverController.getHID().setRumble(RumbleType.kLeftRumble, intensity);
    m_driverController.getHID().setRumble(RumbleType.kRightRumble, intensity);
  }

  private Command driverRumblePulseCommand(double intensity, double durationSec) {
    return Commands.startEnd(
        () -> setDriverRumble(intensity),
        () -> setDriverRumble(0.0))
        .withTimeout(durationSec)
        .ignoringDisable(true);
  }

  private Trigger testModeTrigger(Trigger trigger) {
    return new Trigger(DriverStation::isTestEnabled).and(trigger);
  }

  private Trigger nonTestModeTrigger(Trigger trigger) {
    return new Trigger(() -> !DriverStation.isTestEnabled()).and(trigger);
  }

  private Trigger inactiveLeadWarningTrigger(DoubleSupplier leadSecSupplier) {
    return new Trigger(() -> matchTimingService.timingKnown()
        && !matchTimingService.zoneActive()
        && matchTimingService.zoneRemainingSec() > 0.0
        && matchTimingService.zoneRemainingSec() <= leadSecSupplier.getAsDouble());
  }

  private void configureBindings() {
    // ==================== TRIGGERS ====================

    // Left Trigger: Agitate intake (hold to agitate to high angle)
    m_driverController.leftTrigger().whileTrue(m_superstructure.agitateIntakeCommand());

    // Right Trigger: Auto-Aim & Shoot/Feed
    m_driverController.rightTrigger().whileTrue(
        m_superstructure.aimAndShootCommand(
            () -> getShootTranslationSpeed(m_driverController.getLeftY()),
            () -> getShootTranslationSpeed(m_driverController.getLeftX()),
            true));

    // ==================== BUMPERS ====================

    // Left Bumper: Toggle intake deployed/retracted
    m_driverController.leftBumper().onTrue(m_superstructure.toggleIntakeCommand());

    // ==================== FACE BUTTONS ====================

    // X: X-lock wheels (defensive stance)
    m_driverController.x().whileTrue(m_swerve.applyRequest(() -> xLock));

    // B: Feed (500 RPM, max hood angle)
    m_driverController.b().whileTrue(m_superstructure.feedCommand());
    // Start: Toggle intake to deploy-high (+10 deg) / retract
    m_driverController.start().onTrue(m_superstructure.toggleIntakeHighCommand());

    // Back: Hold to run wheel radius characterization spin test
    m_driverController.back().whileTrue(new WheelRadiusCharacterization(m_swerve));

    // A: Debug - Run hopper while held
    m_driverController.a().whileTrue(m_superstructure.getHopper().runCommand());

    // Y: Debug - Spin up shooter + set hood from dashboard, feed once ready
    AtomicBoolean debugYShotReadyLatched = new AtomicBoolean(false);
    m_driverController.y().whileTrue(
        Commands.run(
            () -> {
              double requestedRpm = SmartDashboard.getNumber(kDebugYShotRpmKey, kDebugYShotDefaultRpm);
              double requestedHoodDeg = SmartDashboard.getNumber(kDebugYShotHoodDegKey, kDebugYShotDefaultHoodDeg);
              double rpm = Math.max(0.0, requestedRpm);
              double hoodDeg = requestedHoodDeg;
              double clampedHoodDeg = Math.max(
                  ShooterConstants.kMinHoodAngle.in(Degrees),
                  Math.min(ShooterConstants.kMaxHoodAngle.in(Degrees), hoodDeg));
              double targetRps = rpm / 60.0;
              double distanceMeters = AutoAim.getDistanceMeters(m_swerve.getPose());

              m_superstructure.getShooter().setFlywheelSpeed(targetRps);
              m_superstructure.getShooter().setHoodAngle(
                  ShooterConstants.launchRadToMechanismRotations(Math.toRadians(clampedHoodDeg)));

              boolean shooterAtSetpoint = m_superstructure.getShooter().isAtSetpoint();
              if (shooterAtSetpoint) {
                debugYShotReadyLatched.set(true);
              }

              m_superstructure.getIntake().setAgitating(true);
              m_superstructure.getShooter().setFiring(shooterAtSetpoint);
              m_superstructure.getHopper().setRunning(debugYShotReadyLatched.get());

              telemetry.publishYShotState(new YShotSnapshot(
                  true,
                  requestedRpm / 60.0,
                  Math.toRadians(requestedHoodDeg),
                  rpm / 60.0,
                  Math.toRadians(clampedHoodDeg),
                  distanceMeters,
                  debugYShotReadyLatched.get()));
            },
            m_superstructure.getShooter(),
            m_superstructure.getHopper(),
            m_superstructure.getIntake())
            .beforeStarting(() -> debugYShotReadyLatched.set(false))
            .finallyDo(() -> {
              m_superstructure.getShooter().stop();
              m_superstructure.getHopper().stop();
              m_superstructure.getIntake().setAgitating(false);
              debugYShotReadyLatched.set(false);
              telemetry.publishYShotState(new YShotSnapshot(
                  false,
                  0.0,
                  0.0,
                  0.0,
                  0.0,
                  0.0,
                  false));
            }));

    // D-pad Up: Stage a note normally, or run the full shooter characterization in
    // Test mode.
    nonTestModeTrigger(m_driverController.povUp()).onTrue(m_superstructure.stagePreloadCommand());
    testModeTrigger(m_driverController.povUp()).onTrue(
        createFullShooterCharacterizationCommand("Shooter Full Characterization"));

    // D-pad Left/Down/Right: Individual shooter characterization shortcuts in Test
    // mode.
    testModeTrigger(m_driverController.povLeft()).onTrue(
        createShooterCharacterizationCommand(
            ShooterCharacterizationMode.VELOCITY_HOLD_SWEEP,
            "Shooter Velocity Hold Sweep"));
    testModeTrigger(m_driverController.povDown()).onTrue(
        createShooterCharacterizationCommand(
            ShooterCharacterizationMode.CLOSED_LOOP_STEP_SWEEP,
            "Shooter Closed-Loop Step Sweep"));
    testModeTrigger(m_driverController.povRight()).onTrue(
        createShooterCharacterizationCommand(
            ShooterCharacterizationMode.OPEN_LOOP_VOLTAGE_SWEEP,
            "Shooter Open-Loop Voltage Sweep"));

    // Right Stick (press): Low-priority manual re-home for intake + hood
    m_driverController.rightStick().onTrue(m_superstructure.homeIntakeAndHoodCommand());

    // Left Stick (press): Stop shooter characterization in Test mode.
    testModeTrigger(m_driverController.leftStick()).onTrue(
        Commands.runOnce(() -> m_superstructure.getShooter().stopCharacterization(), m_superstructure.getShooter())
            .withName("Shooter Characterization Stop"));

    inactiveLeadWarningTrigger(this::getActiveWindowEarlyRumbleLeadSec)
        .onTrue(driverRumblePulseCommand(kActiveWindowEarlyRumbleIntensity, kActiveWindowRumbleSec)
            .withName("Driver Rumble Active Window Lead 10s"));

    inactiveLeadWarningTrigger(this::getActiveWindowRumbleLeadSec)
        .onTrue(driverRumblePulseCommand(kActiveWindowRumbleIntensity, kActiveWindowRumbleSec)
            .withName("Driver Rumble Active Window Lead"));

    // ==================== DEFAULT COMMANDS ====================
    m_swerve.setDefaultCommand(
        m_swerve.applyRequest(() -> {
          double vX = -m_driverController.getLeftY() * MaxSpeed;
          double vY = -m_driverController.getLeftX() * MaxSpeed;
          double omega = getDefaultDriveRotationRate(vX, vY, m_driverController.getRightX());
          return drive
              .withVelocityX(vX)
              .withVelocityY(vY)
              .withRotationalRate(omega);
        }));
  }

  private void configureAutonomous() {
    m_autoChooser.addRoutine("Do Nothing", m_autos::doNothing);
    m_autoChooser.addRoutine("rightTrench", m_autos::rightTrench);
    m_autoChooser.addRoutine("leftTrenchPrototype", m_autos::leftTrenchPrototype);
    m_autoChooser.addRoutine("rightTrenchPrototype", m_autos::rightTrenchPrototype);
    m_autoChooser.addRoutine("shootPreloaded", m_autos::shootPreloaded);
    m_autoChooser.addRoutine("speed", m_autos::speed);
    m_autoChooser.addRoutine("leftBump", m_autos::leftBump);
    m_autoChooser.addRoutine("leftBumpDepot", m_autos::leftBumpDepot);
    m_autoChooser.addRoutine("rightBump", m_autos::rightBump);
    m_autoChooser.select("Do Nothing");
    SmartDashboard.putData("AutoChooser", m_autoChooser);
  }

  public Command getAutonomousCommand() {
    return m_autoChooser.selectedCommandScheduler();
  }

  private Command createShooterCharacterizationCommand(
      ShooterCharacterizationMode mode,
      String name) {
    return Commands.sequence(
        Commands.runOnce(() -> {
          m_superstructure.getHopper().stop();
          m_superstructure.getIntake().setAgitating(false);
          m_superstructure.getShooter().startCharacterization(shooterCharacterizationConfig, mode);
        }, m_superstructure.getShooter(), m_superstructure.getHopper(), m_superstructure.getIntake()),
        Commands.run(() -> {
          m_superstructure.getHopper().stop();
          m_superstructure.getIntake().setAgitating(false);
        }, m_superstructure.getShooter(), m_superstructure.getHopper(), m_superstructure.getIntake())
            .until(() -> !m_superstructure.getShooter().isCharacterizing()))
        .finallyDo(interrupted -> {
          m_superstructure.getShooter().stopCharacterization();
          m_superstructure.getHopper().stop();
          m_superstructure.getIntake().setAgitating(false);
        })
        .withName(name);
  }

  private Command createFullShooterCharacterizationCommand(String name) {
    Command velocitySweep = createShooterCharacterizationCommand(
        ShooterCharacterizationMode.VELOCITY_HOLD_SWEEP,
        "Shooter Velocity Hold Sweep");
    Command stepSweep = createShooterCharacterizationCommand(
        ShooterCharacterizationMode.CLOSED_LOOP_STEP_SWEEP,
        "Shooter Closed-Loop Step Sweep");

    return Commands.sequence(
        velocitySweep,
        Commands.waitSeconds(0.5),
        Commands.either(stepSweep, Commands.none(),
            () -> !m_superstructure.getShooter().getCharacterizationStatus().aborted()))
        .withName(name);
  }
}
