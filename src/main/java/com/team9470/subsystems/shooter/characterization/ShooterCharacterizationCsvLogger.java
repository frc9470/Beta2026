package com.team9470.subsystems.shooter.characterization;

import com.team9470.telemetry.structs.ShooterCharacterizationSnapshot;
import java.io.BufferedWriter;
import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.time.LocalDateTime;
import java.time.format.DateTimeFormatter;

public final class ShooterCharacterizationCsvLogger implements AutoCloseable {
    static final String HEADER = String.join(",",
            "timestamp_sec",
            "run_id",
            "mode",
            "segment_index",
            "commanded_rpm",
            "commanded_volts",
            "measured_rpm",
            "battery_volts",
            "flywheel_avg_motor_volts",
            "flywheel_total_supply_current_amps",
            "flywheel_avg_stator_current_amps",
            "electrical_power_watts",
            "velocity_error_rpm",
            "at_setpoint",
            "settled",
            "aborted");

    private static final DateTimeFormatter kFileTimestampFormat = DateTimeFormatter.ofPattern("yyyyMMdd-HHmmss");

    private final BufferedWriter writer;

    private ShooterCharacterizationCsvLogger(BufferedWriter writer) {
        this.writer = writer;
    }

    public static ShooterCharacterizationCsvLogger create(
            int runId,
            ShooterCharacterizationMode mode) throws IOException {
        Path directory = resolveLogDirectory();
        Files.createDirectories(directory);
        String filename = "%s-%04d-%s.csv".formatted(
                LocalDateTime.now().format(kFileTimestampFormat),
                runId,
                mode.name().toLowerCase());
        BufferedWriter writer = Files.newBufferedWriter(
                directory.resolve(filename),
                StandardCharsets.UTF_8);
        ShooterCharacterizationCsvLogger logger = new ShooterCharacterizationCsvLogger(writer);
        logger.writer.write(HEADER);
        logger.writer.newLine();
        logger.writer.flush();
        return logger;
    }

    public static Path resolveLogDirectory() {
        String override = System.getProperty("shooter.char.logDir");
        if (override != null && !override.isBlank()) {
            return Path.of(override);
        }
        return Path.of("/home/lvuser/logs/shooter-char");
    }

    public void append(ShooterCharacterizationMode mode, ShooterCharacterizationSnapshot snapshot) throws IOException {
        writer.write(toCsvRow(mode, snapshot));
        writer.newLine();
    }

    @Override
    public void close() throws IOException {
        writer.flush();
        writer.close();
    }

    public static String toCsvRow(ShooterCharacterizationMode mode, ShooterCharacterizationSnapshot snapshot) {
        return String.join(",",
                Double.toString(snapshot.timestampSec()),
                Integer.toString(snapshot.runId()),
                mode.name(),
                Integer.toString(snapshot.segmentIndex()),
                Double.toString(snapshot.commandedRpm()),
                Double.toString(snapshot.commandedVolts()),
                Double.toString(snapshot.measuredRpm()),
                Double.toString(snapshot.batteryVolts()),
                Double.toString(snapshot.flywheelAvgMotorVolts()),
                Double.toString(snapshot.flywheelTotalSupplyCurrentAmps()),
                Double.toString(snapshot.flywheelAvgStatorCurrentAmps()),
                Double.toString(snapshot.electricalPowerWatts()),
                Double.toString(snapshot.velocityErrorRpm()),
                Boolean.toString(snapshot.atSetpoint()),
                Boolean.toString(snapshot.settled()),
                Boolean.toString(snapshot.aborted()));
    }
}
