package com.team9470.commands;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;

import com.team9470.bline.FollowPath;
import com.team9470.bline.JsonUtils;
import com.team9470.bline.Path;
import com.team9470.subsystems.Superstructure;
import com.team9470.subsystems.swerve.Swerve;
import com.team9470.telemetry.TelemetryManager;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WrapperCommand;

public class Autos {
  private final AutoFactory m_autoFactory;
  private final FollowPath.Builder m_bLineBuilder;
  private final Swerve m_swerve;
  private final TelemetryManager telemetry = TelemetryManager.getInstance();

  private static final double kAutoAgitateDelaySec = 1.0;
  private static final double kTrenchShotTimeoutSec = 3.0;
  private static final double kBumpShotTimeoutSec = 3.25;

  public Autos(Swerve swerve) {
    m_swerve = swerve;
    m_autoFactory = swerve.createAutoFactory();
    m_bLineBuilder = new FollowPath.Builder(
        swerve,
        swerve::getPose,
        swerve::getChassisSpeeds,
        swerve::setChassisSpeeds,
        new PIDController(5.0, 0.0, 0.0), // Translation PID (tune on robot)
        new PIDController(3.0, 0.0, 0.0), // Rotation PID (tune on robot)
        new PIDController(2.0, 0.0, 0.0) // Cross-track PID (tune on robot)
    ).withDefaultShouldFlip()
        .withPoseReset(swerve::resetPose);
  }

  /**
   * Wraps aimAndShootCommand with a 1-second delayed agitation running in
   * parallel.
   * When the shoot command finishes or times out, agitation is automatically
   * cancelled.
   */
  private static Command aimShootWithAgitate(double timeoutSec) {
    return Commands.deadline(
        Superstructure.getInstance().aimAndShootCommand().withTimeout(timeoutSec),
        Commands.waitSeconds(kAutoAgitateDelaySec)
            .andThen(Superstructure.getInstance().agitateIntakeCommand()));
  }

  private static void bindAutoStaging(AutoTrajectory trajectory) {
    trajectory.atTime("staging")
        .onTrue(Superstructure.getInstance().stagePreloadAutoCommand());
  }

  private Command instrumentSequenceStart(Command command) {
    return new WrapperCommand(command) {
      @Override
      public void initialize() {
        telemetry.markDriveAutoSequenceStart(Timer.getTimestamp());
        super.initialize();
      }
    };
  }

  private Command startFirstTrajectory(AutoTrajectory trajectory) {
    return new WrapperCommand(trajectory.cmd()) {
      @Override
      public void initialize() {
        telemetry.markDriveAutoResetOdometry(Timer.getTimestamp());
        m_swerve.resetPose(trajectory.getInitialPose()
            .orElseThrow(() -> new IllegalStateException("First auto trajectory has no initial pose")));
        telemetry.markDriveAutoDeployIntake(Timer.getTimestamp());
        Superstructure.getInstance().getIntake().setDeployed(true);
        telemetry.markDriveAutoFirstTrajectoryCommandInit(Timer.getTimestamp());
        super.initialize();
      }
    };
  }

  private void bindRoutineStartup(AutoRoutine routine, Command autoCommand) {
    routine.observe(DriverStation::isEnabled)
        .onTrue(Commands.runOnce(() -> telemetry.markDriveAutoRoutineFirstPoll(Timer.getTimestamp())));
    routine.observe(DriverStation::isEnabled)
        .onTrue(Commands.runOnce(() -> telemetry.markDriveAutoRoutineStartTrigger(Timer.getTimestamp())));
    routine.observe(DriverStation::isEnabled).onTrue(instrumentSequenceStart(autoCommand));
  }

  private static AutoTrajectory loadTrajectory(
      AutoRoutine routine, String trajectoryName, boolean mirrorAcrossY) {
    String resolvedTrajectoryName = resolveTrajectoryOverride(trajectoryName);
    AutoTrajectory trajectory = routine.trajectory(resolvedTrajectoryName);
    return mirrorAcrossY ? trajectory.mirrorY() : trajectory;
  }

  private static String resolveTrajectoryOverride(String trajectoryName) {
    if (!RobotBase.isSimulation()) {
      return trajectoryName;
    }
    return System.getProperty("autoSim.override." + trajectoryName, trajectoryName);
  }

  private AutoRoutine buildTrenchRoutine(
      String routineName, boolean mirrorAcrossY, boolean driveToCenter) {
    AutoRoutine routine = m_autoFactory.newRoutine(routineName);
    AutoTrajectory firstCycle = loadTrajectory(routine, "leftTrenchCycle1", mirrorAcrossY);
    AutoTrajectory secondCycle = loadTrajectory(routine, "leftTrenchCycle2", mirrorAcrossY);

    Command autoCommand = startFirstTrajectory(firstCycle)
        .andThen(aimShootWithAgitate(kTrenchShotTimeoutSec))
        .andThen(secondCycle.cmd())
        .andThen(aimShootWithAgitate(kTrenchShotTimeoutSec));

    if (driveToCenter) {
      AutoTrajectory toCenter = loadTrajectory(routine, "leftTrenchToCenter", mirrorAcrossY);
      autoCommand = autoCommand.andThen(toCenter.cmd());
    }

    bindRoutineStartup(routine, autoCommand);
    return routine;
  }

  private AutoRoutine buildBumpRoutine(
      String routineName,
      boolean mirrorAcrossY,
      String finishTrajectoryName,
      boolean shootAfterFinish) {
    AutoRoutine routine = m_autoFactory.newRoutine(routineName);
    AutoTrajectory firstCycle = loadTrajectory(routine, "leftBumpCycle1", mirrorAcrossY);
    AutoTrajectory secondCycle = loadTrajectory(routine, "leftBumpCycle2", mirrorAcrossY);
    AutoTrajectory finishTrajectory = loadTrajectory(routine, finishTrajectoryName, mirrorAcrossY);

    Command autoCommand = startFirstTrajectory(firstCycle)
        .andThen(aimShootWithAgitate(kBumpShotTimeoutSec))
        .andThen(secondCycle.cmd())
        .andThen(aimShootWithAgitate(kBumpShotTimeoutSec))
        .andThen(finishTrajectory.cmd());

    if (shootAfterFinish) {
      autoCommand = autoCommand.andThen(aimShootWithAgitate(kBumpShotTimeoutSec));
    }

    bindRoutineStartup(routine, autoCommand);

    bindAutoStaging(firstCycle);
    bindAutoStaging(secondCycle);
    return routine;
  }

  public AutoRoutine doNothing() {
    AutoRoutine routine = m_autoFactory.newRoutine("DoNothing");
    bindRoutineStartup(routine, Commands.none());
    return routine;
  }

  public AutoRoutine rightTrench() {
    return buildTrenchRoutine("rightTrench", true, true);
  }

  public AutoRoutine leftTrench() {
    return buildTrenchRoutine("leftTrench", false, true);
  }

  public AutoRoutine shootPreloaded() {
    AutoRoutine routine = m_autoFactory.newRoutine("shootPreloaded");
    bindRoutineStartup(routine, aimShootWithAgitate(5));
    return routine;
  }

  public AutoRoutine leftBump() {
    return buildBumpRoutine("leftBump", false, "overLeftBump", false);
  }

  public AutoRoutine leftBumpDepot() {
    return buildBumpRoutine("leftBumpDepot", false, "leftBumpDepot", true);
  }

  public AutoRoutine rightBump() {
    return buildBumpRoutine("rightBump", true, "overLeftBump", false);
  }

  public AutoRoutine leftBumpConservative() {
    AutoRoutine routine = m_autoFactory.newRoutine("leftBumpConservative");
    AutoTrajectory firstCycle = loadTrajectory(routine, "leftBumpCycle1", false);
    AutoTrajectory secondCycle = loadTrajectory(routine, "leftBumpCycle2Conservative", false);
    AutoTrajectory finishTrajectory = loadTrajectory(routine, "overLeftBump", false);

    Command autoCommand = startFirstTrajectory(firstCycle)
        .andThen(aimShootWithAgitate(kBumpShotTimeoutSec))
        .andThen(secondCycle.cmd())
        .andThen(aimShootWithAgitate(kBumpShotTimeoutSec))
        .andThen(finishTrajectory.cmd());

    bindRoutineStartup(routine, autoCommand);

    bindAutoStaging(firstCycle);
    bindAutoStaging(secondCycle);
    return routine;
  }

  public AutoRoutine leftBumpRush() {
    AutoRoutine routine = m_autoFactory.newRoutine("leftBumpRush");
    AutoTrajectory firstCycle = loadTrajectory(routine, "leftBumpCycle1Rush", false);
    AutoTrajectory secondCycle = loadTrajectory(routine, "leftBumpCycle2", false);
    AutoTrajectory finishTrajectory = loadTrajectory(routine, "overLeftBump", false);

    Command autoCommand = startFirstTrajectory(firstCycle)
        .andThen(aimShootWithAgitate(kBumpShotTimeoutSec))
        .andThen(secondCycle.cmd())
        .andThen(aimShootWithAgitate(kBumpShotTimeoutSec))
        .andThen(finishTrajectory.cmd());

    bindRoutineStartup(routine, autoCommand);

    bindAutoStaging(firstCycle);
    bindAutoStaging(secondCycle);
    return routine;
  }

  public AutoRoutine rightBumpRush() {
    AutoRoutine routine = m_autoFactory.newRoutine("rightBumpRush");
    AutoTrajectory firstCycle = loadTrajectory(routine, "leftBumpCycle1Rush", true);
    AutoTrajectory secondCycle = loadTrajectory(routine, "leftBumpCycle2", true);
    AutoTrajectory finishTrajectory = loadTrajectory(routine, "overLeftBump", true);

    Command autoCommand = startFirstTrajectory(firstCycle)
        .andThen(aimShootWithAgitate(kBumpShotTimeoutSec))
        .andThen(secondCycle.cmd())
        .andThen(aimShootWithAgitate(kBumpShotTimeoutSec))
        .andThen(finishTrajectory.cmd());

    bindRoutineStartup(routine, autoCommand);

    bindAutoStaging(firstCycle);
    bindAutoStaging(secondCycle);
    return routine;
  }

  public AutoRoutine rightBumpConservative() {
    AutoRoutine routine = m_autoFactory.newRoutine("rightBumpConservative");
    AutoTrajectory firstCycle = loadTrajectory(routine, "leftBumpCycle1", true);
    AutoTrajectory secondCycle = loadTrajectory(routine, "leftBumpCycle2Conservative", true);
    AutoTrajectory finishTrajectory = loadTrajectory(routine, "overLeftBump", true);

    Command autoCommand = startFirstTrajectory(firstCycle)
        .andThen(aimShootWithAgitate(kBumpShotTimeoutSec))
        .andThen(secondCycle.cmd())
        .andThen(aimShootWithAgitate(kBumpShotTimeoutSec))
        .andThen(finishTrajectory.cmd());

    bindRoutineStartup(routine, autoCommand);

    bindAutoStaging(firstCycle);
    bindAutoStaging(secondCycle);
    return routine;
  }

  public AutoRoutine leftBumpPrototype() {
    AutoRoutine routine = m_autoFactory.newRoutine("leftBumpPrototype");
    AutoTrajectory firstCycle = loadTrajectory(routine, "leftBumpCycle1", false);
    AutoTrajectory secondCycle = loadTrajectory(routine, "leftBumpCycle2Prototype", false);
    AutoTrajectory finishTrajectory = loadTrajectory(routine, "overLeftBump", false);

    Command autoCommand = startFirstTrajectory(firstCycle)
        .andThen(aimShootWithAgitate(kBumpShotTimeoutSec))
        .andThen(secondCycle.cmd())
        .andThen(aimShootWithAgitate(kBumpShotTimeoutSec))
        .andThen(finishTrajectory.cmd());

    bindRoutineStartup(routine, autoCommand);

    bindAutoStaging(firstCycle);
    bindAutoStaging(secondCycle);
    return routine;
  }

  public AutoRoutine rightBumpPrototype() {
    AutoRoutine routine = m_autoFactory.newRoutine("rightBumpPrototype");
    AutoTrajectory firstCycle = loadTrajectory(routine, "leftBumpCycle1", true);
    AutoTrajectory secondCycle = loadTrajectory(routine, "leftBumpCycle2Prototype", true);
    AutoTrajectory finishTrajectory = loadTrajectory(routine, "overLeftBump", true);

    Command autoCommand = startFirstTrajectory(firstCycle)
        .andThen(aimShootWithAgitate(kBumpShotTimeoutSec))
        .andThen(secondCycle.cmd())
        .andThen(aimShootWithAgitate(kBumpShotTimeoutSec))
        .andThen(finishTrajectory.cmd());

    bindRoutineStartup(routine, autoCommand);

    bindAutoStaging(firstCycle);
    bindAutoStaging(secondCycle);
    return routine;
  }

  // ==================== BLINE AUTO ROUTINES ====================

  /**
   * Simple BLine test auto — drives 2m forward at (1.5, 5.5) → (3.5, 5.5).
   * Use this to validate BLine integration and PID tuning.
   */
  public Command blineTest() {
    Path testPath = JsonUtils.loadPath("blineTest.json");
    return m_bLineBuilder.build(testPath).withName("BLine Test");
  }
}
