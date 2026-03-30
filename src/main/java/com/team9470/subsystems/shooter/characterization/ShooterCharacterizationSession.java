package com.team9470.subsystems.shooter.characterization;

import java.util.ArrayList;
import java.util.List;

public final class ShooterCharacterizationSession {
    private static final double kIdleHoldSec = 1.0;

    private final int runId;
    private final ShooterCharacterizationMode mode;
    private final List<Segment> segments;
    private int segmentIndex = 0;
    private double segmentStartSec;
    private boolean complete = false;
    private boolean aborted = false;
    private String abortReason = "";

    public ShooterCharacterizationSession(
            int runId,
            ShooterCharacterizationMode mode,
            ShooterCharacterizationConfig config,
            double startTimestampSec) {
        this.runId = runId;
        this.mode = mode;
        this.segments = buildSegments(mode, config);
        this.segmentStartSec = startTimestampSec;
        if (segments.isEmpty()) {
            abort("No characterization segments configured");
        }
    }

    public Sample update(double timestampSec, boolean atSetpoint) {
        if (complete || aborted) {
            return Sample.inactive(runId, mode, segmentIndex, segments.size(), aborted, abortReason);
        }

        while (!complete && !aborted) {
            Segment segment = segments.get(segmentIndex);
            double elapsedSec = Math.max(0.0, timestampSec - segmentStartSec);
            boolean settled = segment.settled(elapsedSec, atSetpoint);
            if (elapsedSec >= segment.durationSec()) {
                if (segmentIndex >= segments.size() - 1) {
                    complete = true;
                    return new Sample(
                            runId,
                            mode,
                            segmentIndex,
                            segments.size(),
                            segment.commandedRpm(),
                            segment.commandedVolts(),
                            false,
                            true,
                            false,
                            false,
                            "",
                            elapsedSec);
                }
                segmentIndex++;
                segmentStartSec += segment.durationSec();
                continue;
            }

            return new Sample(
                    runId,
                    mode,
                    segmentIndex,
                    segments.size(),
                    segment.commandedRpm(),
                    segment.commandedVolts(),
                    true,
                    false,
                    settled,
                    false,
                    "",
                    elapsedSec);
        }

        return Sample.inactive(runId, mode, segmentIndex, segments.size(), aborted, abortReason);
    }

    public void abort(String reason) {
        aborted = true;
        abortReason = reason == null ? "" : reason;
    }

    public boolean isActive() {
        return !complete && !aborted;
    }

    public boolean isComplete() {
        return complete;
    }

    public boolean isAborted() {
        return aborted;
    }

    public ShooterCharacterizationStatus status() {
        if (segments.isEmpty()) {
            return new ShooterCharacterizationStatus(false, complete, aborted, runId, mode, -1, 0, 0.0, 0.0, abortReason);
        }
        int clampedIndex = Math.max(0, Math.min(segmentIndex, segments.size() - 1));
        Segment segment = segments.get(clampedIndex);
        return new ShooterCharacterizationStatus(
                isActive(),
                complete,
                aborted,
                runId,
                mode,
                clampedIndex,
                segments.size(),
                segment.commandedRpm(),
                segment.commandedVolts(),
                abortReason);
    }

    private static List<Segment> buildSegments(
            ShooterCharacterizationMode mode,
            ShooterCharacterizationConfig config) {
        List<Segment> segments = new ArrayList<>();
        switch (mode) {
            case VELOCITY_HOLD_SWEEP -> {
                double[] plateaus = config.rpmPlateaus();
                for (double rpm : plateaus) {
                    segments.add(new Segment(rpm, 0.0, config.plateauDwellSec(), config.plateauDiscardSec(), true));
                }
                for (int i = plateaus.length - 2; i >= 0; i--) {
                    segments.add(new Segment(plateaus[i], 0.0, config.plateauDwellSec(), config.plateauDiscardSec(), true));
                }
            }
            case OPEN_LOOP_VOLTAGE_SWEEP -> {
                for (double volts : config.voltageSteps()) {
                    segments.add(new Segment(0.0, volts, config.voltageStepHoldSec(), 0.0, false));
                    segments.add(new Segment(0.0, 0.0, config.voltageStepHoldSec(), 0.0, false));
                }
            }
            case CLOSED_LOOP_STEP_SWEEP -> {
                for (double targetRpm : config.shotTargetsRpm()) {
                    for (double idleRpm : config.idleCandidatesRpm()) {
                        segments.add(new Segment(idleRpm, 0.0, kIdleHoldSec, 0.0, true));
                        segments.add(new Segment(targetRpm, 0.0, config.stepTimeoutSec(), 0.0, true));
                    }
                }
            }
            case DISABLED -> {
            }
        }
        return segments;
    }

    public record Segment(
            double commandedRpm,
            double commandedVolts,
            double durationSec,
            double settleAfterSec,
            boolean requiresAtSetpoint) {
        boolean settled(double elapsedSec, boolean atSetpoint) {
            if (elapsedSec < settleAfterSec) {
                return false;
            }
            return !requiresAtSetpoint || atSetpoint;
        }
    }

    public record Sample(
            int runId,
            ShooterCharacterizationMode mode,
            int segmentIndex,
            int totalSegments,
            double commandedRpm,
            double commandedVolts,
            boolean active,
            boolean complete,
            boolean settled,
            boolean aborted,
            String abortReason,
            double elapsedSegmentSec) {
        static Sample inactive(
                int runId,
                ShooterCharacterizationMode mode,
                int segmentIndex,
                int totalSegments,
                boolean aborted,
                String abortReason) {
            return new Sample(
                    runId,
                    mode,
                    segmentIndex,
                    totalSegments,
                    0.0,
                    0.0,
                    false,
                    !aborted,
                    false,
                    aborted,
                    abortReason == null ? "" : abortReason,
                    0.0);
        }
    }
}
