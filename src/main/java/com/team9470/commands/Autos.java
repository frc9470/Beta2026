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

    routine.active().onTrue(
        speed.resetOdometry()
            .andThen(speed.cmd())
            .andThen(Superstructure.getInstance().aimAndShootCommand()));

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
            .andThen(Superstructure.getInstance().aimAndShootCommand().withTimeout(4.5))
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
            // .andThen(Commands.waitUntil(() -> Superstructure.getInstance().getIntake()
            // .getPivotAngle() <= IntakeConstants.kDeployAngle.in(Radians) +
            // Math.toRadians(15)))
            .andThen(rightTrench.cmd())
            .andThen(Superstructure.getInstance().aimAndShootCommand().withTimeout(4.5))
            .andThen(rightTrench2.cmd())
            .andThen(Superstructure.getInstance().aimAndShootCommand().withTimeout(4))
            .andThen(rightTrench3.cmd()));
    return routine;
  }

  public AutoRoutine leftTrenchStable() {
    AutoRoutine routine = m_autoFactory.newRoutine("leftTrenchStable");
    AutoTrajectory leftTrench = routine.trajectory("leftTrenchCycle1Prototype");
    AutoTrajectory leftTrench2 = routine.trajectory("leftTrenchCycle2Stable");

    routine.active().onTrue(
        leftTrench.resetOdometry()
            .andThen(new InstantCommand(() -> Superstructure.getInstance().getIntake().setDeployed(true)))
            // .andThen(Commands.waitUntil(() -> Superstructure.getInstance().getIntake()
            // .getPivotAngle() <= IntakeConstants.kDeployAngle.in(Radians) +
            // Math.toRadians(45)))
            .andThen(leftTrench.cmd())
            .andThen(Superstructure.getInstance().aimAndShootCommand().withTimeout(4.5))
            .andThen(leftTrench2.cmd())
            .andThen(Superstructure.getInstance().aimAndShootCommand()));
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
            // .andThen(Commands.waitUntil(() -> Superstructure.getInstance().getIntake()
            // .getPivotAngle() <= IntakeConstants.kDeployAngle.in(Radians) +
            // Math.toRadians(15)))
            .andThen(leftTrench.cmd())
            .andThen(Superstructure.getInstance().aimAndShootCommand().withTimeout(4.25))
            .andThen(leftTrench2.cmd())
            .andThen(Superstructure.getInstance().aimAndShootCommand().withTimeout(4))
            .andThen(leftTrench3.cmd()));
    return routine;
  }

  public AutoRoutine bumpRightBlue() {
    AutoRoutine routine = m_autoFactory.newRoutine("bumpRightBlue");
    AutoTrajectory bumpRightBlue = routine.trajectory("bumpRightBlue");

    routine.active().onTrue(
        bumpRightBlue.resetOdometry()
            .andThen(bumpRightBlue.cmd())
            .andThen(Superstructure.getInstance().aimAndShootCommand().withTimeout(5)));

    bumpRightBlue.atTime("IntakeDown")
        .onTrue(new InstantCommand(() -> Superstructure.getInstance().getIntake().setDeployed(true)));
    bumpRightBlue.atTime("IntakeUp")
        .onTrue(new InstantCommand(() -> Superstructure.getInstance().getIntake().setDeployed(false)));
    return routine;
  }

  public AutoRoutine bumpLeftBlue() {
    AutoRoutine routine = m_autoFactory.newRoutine("bumpLeftBlue");
    AutoTrajectory bumpLeftBlue = routine.trajectory("bumpLeftBlue");

    routine.active().onTrue(
        bumpLeftBlue.resetOdometry()
            .andThen(bumpLeftBlue.cmd())
            .andThen(Superstructure.getInstance().aimAndShootCommand().withTimeout(5)));

    bumpLeftBlue.atTime("IntakeDown")
        .onTrue(new InstantCommand(() -> Superstructure.getInstance().getIntake().setDeployed(true)));
    bumpLeftBlue.atTime("IntakeUp")
        .onTrue(new InstantCommand(() -> Superstructure.getInstance().getIntake().setDeployed(false)));
    return routine;
  }

  public AutoRoutine depotOutpostBlue() {
    AutoRoutine routine = m_autoFactory.newRoutine("depotOutpostBlue");
    AutoTrajectory depotOutpostBlue = routine.trajectory("depotOutpostBlue");

    routine.active().onTrue(
        depotOutpostBlue.resetOdometry()
            .andThen(depotOutpostBlue.cmd())
            .andThen(Superstructure.getInstance().aimAndShootCommand()));

    depotOutpostBlue.atTime("IntakeDown")
        .onTrue(new InstantCommand(() -> Superstructure.getInstance().getIntake().setDeployed(true)));
    return routine;
  }

  public AutoRoutine driveOverBumpTest() {
    AutoRoutine routine = m_autoFactory.newRoutine("driveOverBumpTest");
    AutoTrajectory driveOverBumpTest = routine.trajectory("driveOverBumpTest");

    routine.active().onTrue(
        driveOverBumpTest.resetOdometry()
            .andThen(driveOverBumpTest.cmd()));
    return routine;
  }

  public AutoRoutine shootPreloaded() {
    AutoRoutine routine = m_autoFactory.newRoutine("shootPreloaded");
    routine.active().onTrue(Superstructure.getInstance().aimAndShootCommand().withTimeout(5));
    return routine;
  }

  public AutoRoutine outpostIntake() {
    AutoRoutine routine = m_autoFactory.newRoutine("outpostIntake");
    AutoTrajectory outpostIntake1 = routine.trajectory("outpostIntake1");
    AutoTrajectory outpostIntake2 = routine.trajectory("outpostIntake2");

    routine.active().onTrue(
        outpostIntake1.resetOdometry()
            .andThen(new InstantCommand(() -> Superstructure.getInstance().getIntake().setDeployed(true)))
            .andThen(outpostIntake1.cmd())
            .andThen(Commands.waitSeconds(3))
            .andThen(outpostIntake2.cmd())
            .andThen(Superstructure.getInstance().aimAndShootCommand()));
    return routine;
  }
}
