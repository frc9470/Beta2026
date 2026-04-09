// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team9470;

import com.team9470.telemetry.MatchTimingService;
import com.team9470.telemetry.PracticeTimerTracker;
import com.team9470.telemetry.TelemetryManager;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.MatchType;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;

public class Robot extends TimedRobot {
  private static final Path kUsbMountPath = Path.of("/u");
  private static final Path kUsbLogDir = kUsbMountPath.resolve("logs");

  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;
  private final MatchTimingService matchTimingService = MatchTimingService.getInstance();
  private final TelemetryManager telemetry = TelemetryManager.getInstance();
  private final PracticeTimerTracker practiceTimerTracker = new PracticeTimerTracker();

  public Robot() {
    super(0.02); // Run loop at 50Hz (Standard)
    RobotController.setBrownoutVoltage(Units.Volts.of(4.6));
    m_robotContainer = new RobotContainer();
  }

  @Override
  public void robotInit() {
    startUsbDataLoggingIfAvailable();
  }

  private void startUsbDataLoggingIfAvailable() {
    try {
      Path usbPath = kUsbMountPath.toRealPath();
      if (!Files.isDirectory(usbPath) || !Files.isWritable(usbPath)) {
        DriverStation.reportWarning("WPILib data logging disabled: /u is not writable", false);
        return;
      }

      Files.createDirectories(kUsbLogDir);
      DataLogManager.start(kUsbLogDir.toString());
      DriverStation.startDataLog(DataLogManager.getLog(), false);
      System.out.println("WPILib data logging enabled at " + kUsbLogDir);
    } catch (IOException ex) {
      DriverStation.reportWarning("WPILib data logging disabled: no valid USB at /u", false);
    }
  }

  @Override
  public void robotPeriodic() {
    MatchType matchType = DriverStation.getMatchType();
    var practiceTimerOutput = practiceTimerTracker.update(new PracticeTimerTracker.DriverStationSample(
        Timer.getFPGATimestamp(),
        matchType,
        DriverStation.isFMSAttached(),
        DriverStation.isDSAttached(),
        DriverStation.isAutonomousEnabled(),
        DriverStation.isTeleopEnabled(),
        DriverStation.isTestEnabled(),
        DriverStation.isDisabled(),
        DriverStation.getMatchTime(),
        DriverStation.getAlliance(),
        DriverStation.getGameSpecificMessage()));
    matchTimingService.update(practiceTimerOutput);
    telemetry.publishPracticeTimerState(
        practiceTimerOutput.snapshot(),
        practiceTimerOutput.phaseLabel(),
        practiceTimerOutput.zoneLabel());

    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(m_autonomousCommand);
    }
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {

    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void testInit() {

    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {
  }

  @Override
  public void simulationInit() {
  }

  @Override
  public void simulationPeriodic() {
  }
}
