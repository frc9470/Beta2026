package com.team9470.commands;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Test;

class AutosSchedulingTest {
    private final CommandScheduler scheduler = CommandScheduler.getInstance();

    @AfterEach
    void cleanup() {
        scheduler.cancelAll();
        scheduler.run();
    }

    @Test
    void futureSuperstructureRequirementCancelsSequentialAuto() {
        var swerve = new TestSubsystem();
        var superstructure = new TestSubsystem();
        var hopper = new TestSubsystem();

        Command auto = Commands.sequence(
                Commands.run(() -> {
                }, swerve).ignoringDisable(true),
                Commands.run(() -> {
                }, superstructure).ignoringDisable(true));
        Command stage = Commands.run(() -> {
        }, superstructure, hopper).ignoringDisable(true);

        scheduler.schedule(auto);
        scheduler.run();
        assertTrue(auto.isScheduled());

        scheduler.schedule(stage);
        scheduler.run();

        assertFalse(auto.isScheduled());
        assertTrue(stage.isScheduled());
    }

    @Test
    void hopperOnlyStageDoesNotCancelSequentialAuto() {
        var swerve = new TestSubsystem();
        var superstructure = new TestSubsystem();
        var hopper = new TestSubsystem();

        Command auto = Commands.sequence(
                Commands.run(() -> {
                }, swerve).ignoringDisable(true),
                Commands.run(() -> {
                }, superstructure).ignoringDisable(true));
        Command stage = Commands.run(() -> {
        }, hopper).ignoringDisable(true);

        scheduler.schedule(auto);
        scheduler.run();
        assertTrue(auto.isScheduled());

        scheduler.schedule(stage);
        scheduler.run();

        assertTrue(auto.isScheduled());
        assertTrue(stage.isScheduled());
    }

    private static final class TestSubsystem extends SubsystemBase {
    }
}
