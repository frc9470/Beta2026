package com.team9470.commands;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;

import com.team9470.subsystems.Superstructure;
import com.team9470.subsystems.swerve.Swerve;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class Autos {
  private final AutoFactory m_autoFactory;

  private static final double kAutoAgitateDelaySec = 1.0;

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

  /**
   * Wraps aimAndShootCommand with constant field-relative velocity (SOTM) and
   * a 1-second delayed agitation running in parallel.
   * Velocity is specified from the Blue-alliance perspective; the command
   * automatically flips for Red alliance.
   *
   * @param vxMps      field-relative X velocity (m/s, Blue perspective)
   * @param vyMps      field-relative Y velocity (m/s, Blue perspective)
   * @param timeoutSec maximum time to spend shooting on the move
   */
  private static Command sotmShootWithAgitate(double vxMps, double vyMps, double timeoutSec) {
    return Commands.deadline(
        Superstructure.getInstance().aimAndShootCommand(
            () -> vxMps,
            () -> vyMps,
            false).withTimeout(timeoutSec),
        Commands.waitSeconds(kAutoAgitateDelaySec)
            .andThen(Superstructure.getInstance().agitateIntakeCommand()));
  }

  private static void bindAutoStaging(AutoTrajectory trajectory) {
    trajectory.atTime("staging")
        .onTrue(Superstructure.getInstance().stagePreloadAutoCommand());
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
        .onTrue(new InstantCommand(() -> Superstructure.getInstance().getIntake().setDeployed(true)));
    return routine;
  }

  public AutoRoutine rightTrench() {
    AutoRoutine routine = m_autoFactory.newRoutine("rightTrench");
    AutoTrajectory rightTrench = routine.trajectory("rightTrenchCycle1Prototype");
    AutoTrajectory rightTrench2 = routine.trajectory("rightTrenchCycle2");

    routine.active().onTrue(
        rightTrench.resetOdometry()
            .andThen(new InstantCommand(() -> Superstructure.getInstance().getIntake().setDeployed(true)))
            .andThen(rightTrench.cmd())
            .andThen(aimShootWithAgitate(3))
            .andThen(rightTrench2.cmd())
            .andThen(aimShootWithAgitate(3)));
    return routine;
  }

  public AutoRoutine rightTrenchPrototype() {
    AutoRoutine routine = m_autoFactory.newRoutine("rightTrenchPrototype");
    AutoTrajectory rightTrench = routine.trajectory("rightTrenchCycle1Prototype");
    AutoTrajectory rightTrench2 = routine.trajectory("rightTrenchCycle2Prototype");
    AutoTrajectory rightTrench3 = routine.trajectory("rightTrenchToCenter");

    routine.active().onTrue(
        rightTrench.resetOdometry()
            .andThen(new InstantCommand(() -> Superstructure.getInstance().getIntake().setDeployed(true)))
            .andThen(rightTrench.cmd())
            .andThen(aimShootWithAgitate(3))
            .andThen(rightTrench2.cmd())
            .andThen(aimShootWithAgitate(3))
            .andThen(rightTrench3.cmd()));
    return routine;
  }

  public AutoRoutine leftTrenchPrototype() {
    AutoRoutine routine = m_autoFactory.newRoutine("leftTrenchPrototype");
    AutoTrajectory leftTrench = routine.trajectory("leftTrenchCycle1Prototype");
    AutoTrajectory leftTrench2 = routine.trajectory("leftTrenchCycle2Prototype");
    AutoTrajectory leftTrench3 = routine.trajectory("leftTrenchToCenter");

    routine.active().onTrue(
        leftTrench.resetOdometry()
            .andThen(new InstantCommand(() -> Superstructure.getInstance().getIntake().setDeployed(true)))
            .andThen(leftTrench.cmd())
            .andThen(aimShootWithAgitate(3))
            .andThen(leftTrench2.cmd())
            .andThen(aimShootWithAgitate(3))
            .andThen(leftTrench3.cmd()));
    return routine;
  }

  public AutoRoutine shootPreloaded() {
    AutoRoutine routine = m_autoFactory.newRoutine("shootPreloaded");
    routine.active().onTrue(aimShootWithAgitate(5));
    return routine;
  }

  public AutoRoutine leftBump() {
    AutoRoutine routine = m_autoFactory.newRoutine("leftBump");
    AutoTrajectory leftBump = routine.trajectory("leftBumpCycle1");
    AutoTrajectory leftBump2 = routine.trajectory("leftBumpCycle2");
    AutoTrajectory leftBump3 = routine.trajectory("overLeftBump");

    routine.active().onTrue(
        leftBump.resetOdometry()
            .andThen(new InstantCommand(() -> Superstructure.getInstance().getIntake().setDeployed(true)))
            .andThen(leftBump.cmd())
            .andThen(aimShootWithAgitate(3.25))
            .andThen(leftBump2.cmd())
            .andThen(aimShootWithAgitate(3.25))
            .andThen(leftBump3.cmd()));

    bindAutoStaging(leftBump);
    bindAutoStaging(leftBump2);

    return routine;
  }

  public AutoRoutine leftBumpDepot() {
    AutoRoutine routine = m_autoFactory.newRoutine("leftBumpDepot");
    AutoTrajectory leftBump = routine.trajectory("leftBumpCycle1");
    AutoTrajectory leftBump2 = routine.trajectory("leftBumpCycle2");
    AutoTrajectory leftBump3 = routine.trajectory("leftBumpDepot");

    routine.active().onTrue(
        leftBump.resetOdometry()
            .andThen(new InstantCommand(() -> Superstructure.getInstance().getIntake().setDeployed(true)))
            .andThen(leftBump.cmd())
            .andThen(aimShootWithAgitate(3.25))
            .andThen(leftBump2.cmd())
            .andThen(aimShootWithAgitate(3.25))
            .andThen(leftBump3.cmd())
            .andThen(aimShootWithAgitate(3.25)));

    bindAutoStaging(leftBump);
    bindAutoStaging(leftBump2);
    return routine;
  }

  /**
   * Left-bump auto with shoot-on-the-move after the first swipe.
   * Instead of stopping to shoot after cycle 1, the robot fires while moving
   * along the first leg of leftBumpCycle2_copy1 (point 1 -> point 2), then
   * resumes the remainder of cycle 2 and keeps the later shot stationary.
   */
  public AutoRoutine newleftbump() {
    AutoRoutine routine = m_autoFactory.newRoutine("newleftbump");
    AutoTrajectory leftBump = routine.trajectory("leftBumpCycle1");
    AutoTrajectory leftBump2 = routine.trajectory("leftBumpCycle2_copy1", 1);
    AutoTrajectory leftBump3 = routine.trajectory("overLeftBump");

    // SOTM velocity derived from leftBumpCycle2_copy1 point 1 -> point 2:
    // (3.3, 5.5) -> (2.3820, 6.7893) over 0.69042 s.
    // These are Blue-alliance field-relative coordinates and will be flipped
    // automatically for Red by aimAndShootCommand.
    final double sotmVxMps = -1.33;
    final double sotmVyMps = 1.87;
    final double sotmTimeoutSec = 0.69042;

    routine.active().onTrue(
        leftBump.resetOdometry()
            .andThen(new InstantCommand(() -> Superstructure.getInstance().getIntake().setDeployed(true)))
            .andThen(leftBump.cmd())
            .andThen(sotmShootWithAgitate(sotmVxMps, sotmVyMps, sotmTimeoutSec))
            .andThen(leftBump2.cmd())
            .andThen(aimShootWithAgitate(3.25))
            .andThen(leftBump3.cmd()));

    bindAutoStaging(leftBump);
    bindAutoStaging(leftBump2);

    return routine;
  }

  public AutoRoutine rightBump() {
    AutoRoutine routine = m_autoFactory.newRoutine("rightBump");
    AutoTrajectory rightBump = routine.trajectory("rightBumpCycle1");
    AutoTrajectory rightBump2 = routine.trajectory("rightBumpCycle2");
    AutoTrajectory rightBump3 = routine.trajectory("rightBumpToCenter");

    routine.active().onTrue(
        rightBump.resetOdometry()
            .andThen(new InstantCommand(() -> Superstructure.getInstance().getIntake().setDeployed(true)))
            .andThen(rightBump.cmd())
            .andThen(aimShootWithAgitate(3.25))
            .andThen(rightBump2.cmd())
            .andThen(aimShootWithAgitate(3.25))
            .andThen(rightBump3.cmd()));

    bindAutoStaging(rightBump);
    bindAutoStaging(rightBump2);
    return routine;
  }
}
