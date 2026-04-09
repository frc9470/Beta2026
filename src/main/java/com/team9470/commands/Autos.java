package com.team9470.commands;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;

import com.team9470.subsystems.Superstructure;
import com.team9470.subsystems.swerve.Swerve;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class Autos {
  private final AutoFactory m_autoFactory;

  private static final double kAutoAgitateDelaySec = 1.0;
  private static final double kTrenchShotTimeoutSec = 3.0;
  private static final double kBumpShotTimeoutSec = 3.25;

  public Autos(Swerve swerve) {
    m_autoFactory = swerve.createAutoFactory();
  }

  /**
   * Wraps aimAndShootCommand with a 1-second delayed agitation running in parallel.
   * When the shoot command finishes or times out, agitation is automatically cancelled.
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

  private static Command deployIntake() {
    return Commands.runOnce(() -> Superstructure.getInstance().getIntake().setDeployed(true));
  }

  private static AutoTrajectory loadTrajectory(
      AutoRoutine routine, String trajectoryName, boolean mirrorAcrossY) {
    AutoTrajectory trajectory = routine.trajectory(trajectoryName);
    return mirrorAcrossY ? trajectory.mirrorY() : trajectory;
  }

  private AutoRoutine buildTrenchRoutine(
      String routineName, boolean mirrorAcrossY, boolean driveToCenter) {
    AutoRoutine routine = m_autoFactory.newRoutine(routineName);
    AutoTrajectory firstCycle = loadTrajectory(routine, "leftTrenchCycle1", mirrorAcrossY);
    AutoTrajectory secondCycle = loadTrajectory(routine, "leftTrenchCycle2Prototype", mirrorAcrossY);

    Command autoCommand = firstCycle.resetOdometry()
        .andThen(deployIntake())
        .andThen(firstCycle.cmd())
        .andThen(aimShootWithAgitate(kTrenchShotTimeoutSec))
        .andThen(secondCycle.cmd())
        .andThen(aimShootWithAgitate(kTrenchShotTimeoutSec));

    if (driveToCenter) {
      AutoTrajectory toCenter = loadTrajectory(routine, "leftTrenchToCenter", mirrorAcrossY);
      autoCommand = autoCommand.andThen(toCenter.cmd());
    }

    routine.active().onTrue(autoCommand);
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

    Command autoCommand = firstCycle.resetOdometry()
        .andThen(deployIntake())
        .andThen(firstCycle.cmd())
        .andThen(aimShootWithAgitate(kBumpShotTimeoutSec))
        .andThen(secondCycle.cmd())
        .andThen(aimShootWithAgitate(kBumpShotTimeoutSec))
        .andThen(finishTrajectory.cmd());

    if (shootAfterFinish) {
      autoCommand = autoCommand.andThen(aimShootWithAgitate(kBumpShotTimeoutSec));
    }

    routine.active().onTrue(autoCommand);

    bindAutoStaging(firstCycle);
    bindAutoStaging(secondCycle);
    return routine;
  }

  public AutoRoutine doNothing() {
    AutoRoutine routine = m_autoFactory.newRoutine("DoNothing");
    routine.active().onTrue(Commands.none());
    return routine;
  }

  public AutoRoutine speed() {
    AutoRoutine routine = m_autoFactory.newRoutine("speed");
    AutoTrajectory speed = routine.trajectory("speed");
    AutoTrajectory speed2 = routine.trajectory("speed2");
    AutoTrajectory leftTrenchToCenter = routine.trajectory("leftTrenchToCenter");

    routine.active().onTrue(
        speed.resetOdometry()
            .andThen(speed.cmd())
            .andThen(aimShootWithAgitate(3))
            .andThen(speed2.cmd())
            .andThen(aimShootWithAgitate(3.5))
            .andThen(leftTrenchToCenter.cmd()));

    speed.atTime("IntakeDown")
        .onTrue(deployIntake());
    return routine;
  }

  public AutoRoutine rightTrench() {
    return buildTrenchRoutine("rightTrench", true, false);
  }

  public AutoRoutine rightTrenchPrototype() {
    return buildTrenchRoutine("rightTrenchPrototype", true, true);
  }

  public AutoRoutine leftTrenchPrototype() {
    return buildTrenchRoutine("leftTrenchPrototype", false, true);
  }

  public AutoRoutine shootPreloaded() {
    AutoRoutine routine = m_autoFactory.newRoutine("shootPreloaded");
    routine.active().onTrue(aimShootWithAgitate(5));
    return routine;
  }

  public AutoRoutine leftBump() {
    return buildBumpRoutine("leftBump", false, "overLeftBump", false);
  }

  public AutoRoutine leftBumpDepot() {
    return buildBumpRoutine("leftBumpDepot", false, "leftBumpDepot", true);
  }


  public AutoRoutine rightBump() {
    return buildBumpRoutine("rightBump", true, "leftBumpToCenter", false);
  }
}
