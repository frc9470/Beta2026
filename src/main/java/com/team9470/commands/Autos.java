package com.team9470.commands;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;

import com.team9470.subsystems.Superstructure;
import com.team9470.subsystems.swerve.Swerve;
import com.team9470.subsystems.intake.IntakeConstants;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import static edu.wpi.first.units.Units.Radians;

public class Autos {
  private final AutoFactory m_autoFactory;

  public Autos(Swerve swerve) {
    m_autoFactory = swerve.createAutoFactory();
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
    AutoTrajectory goToCenter = routine.trajectory("goToCenter");

    routine.active().onTrue(
        speed.resetOdometry()
            .andThen(speed.cmd())
            .andThen(Superstructure.getInstance().aimAndShootCommand().withTimeout(2.5))
            .andThen(speed2.cmd())
            .andThen(Superstructure.getInstance().aimAndShootCommand().withTimeout(2.5))
            .andThen(goToCenter.cmd()));

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
            .andThen(Superstructure.getInstance().aimAndShootCommand().withTimeout(2.5))
            .andThen(rightTrench2.cmd())
            .andThen(Superstructure.getInstance().aimAndShootCommand()));
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
            .andThen(Superstructure.getInstance().aimAndShootCommand().withTimeout(4.5))
            .andThen(rightTrench2.cmd())
            .andThen(Superstructure.getInstance().aimAndShootCommand().withTimeout(4))
            .andThen(rightTrench3.cmd()));
    return routine;
  }

  public AutoRoutine leftTrenchPrototype() {
    AutoRoutine routine = m_autoFactory.newRoutine("leftTrenchPrototype");
    AutoTrajectory leftTrench = routine.trajectory("leftTrenchCycle1");
    AutoTrajectory leftTrench2 = routine.trajectory("leftTrenchCycle2Prototype");
    AutoTrajectory leftTrench3 = routine.trajectory("leftTrenchToCenter");

    routine.active().onTrue(
        leftTrench.resetOdometry()
            .andThen(new InstantCommand(() -> Superstructure.getInstance().getIntake().setDeployed(true)))
            .andThen(leftTrench.cmd())
            .andThen(Superstructure.getInstance().aimAndShootCommand().withTimeout(2.5))
            .andThen(leftTrench2.cmd())
            .andThen(Superstructure.getInstance().aimAndShootCommand().withTimeout(2.5))
            .andThen(leftTrench3.cmd()));
    return routine;
  }

  public AutoRoutine shootPreloaded() {
    AutoRoutine routine = m_autoFactory.newRoutine("shootPreloaded");
    routine.active().onTrue(Superstructure.getInstance().aimAndShootCommand().withTimeout(5));
    return routine;
  }
}
