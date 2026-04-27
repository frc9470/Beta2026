package com.team9470.subsystems;

import static com.team9470.subsystems.SuperstructureConstants.Aim.*;
import static com.team9470.subsystems.SuperstructureConstants.AutoStage.*;

final class SuperstructureLogic {
    private SuperstructureLogic() {
    }

    static String autoStagePhaseLabel(int phaseCode) {
        return switch (phaseCode) {
            case AUTO_STAGE_PHASE_READY -> "Ready";
            case AUTO_STAGE_PHASE_PROBE_BASELINE -> "ProbeBaseline";
            case AUTO_STAGE_PHASE_PROBE_PULSE -> "ProbePulse";
            case AUTO_STAGE_PHASE_STAGE_COMMAND -> "StageCommand";
            case AUTO_STAGE_PHASE_COMPLETE -> "Complete";
            case AUTO_STAGE_PHASE_IDLE -> "Idle";
            default -> "Unknown";
        };
    }

    static String autoStageReasonLabel(int reasonCode) {
        return switch (reasonCode) {
            case AUTO_STAGE_REASON_NONE -> "None";
            case AUTO_STAGE_REASON_DISABLED -> "Disabled";
            case AUTO_STAGE_REASON_NOT_TELEOP -> "NotTeleop";
            case AUTO_STAGE_REASON_SUPERSTRUCTURE_BUSY -> "SuperstructureBusy";
            case AUTO_STAGE_REASON_HOPPER_BUSY -> "HopperBusy";
            case AUTO_STAGE_REASON_SHOOTER_FIRING -> "ShooterFiring";
            case AUTO_STAGE_REASON_AGITATING -> "Agitating";
            case AUTO_STAGE_REASON_TOP_BLOCKED -> "TopBlocked";
            case AUTO_STAGE_REASON_EVIDENCE_LOW -> "NoLoadEventYet";
            case AUTO_STAGE_REASON_WAITING_FOR_QUIET -> "WaitingForQuiet";
            case AUTO_STAGE_REASON_COOLDOWN -> "Cooldown";
            case AUTO_STAGE_REASON_PROBE_RUNNING -> "ProbeRunning";
            case AUTO_STAGE_REASON_PROBE_REJECTED -> "ProbeRejected";
            case AUTO_STAGE_REASON_STAGE_REQUESTED -> "StageRequested";
            default -> "Unknown";
        };
    }

    static boolean shouldRequestAutoStageFromProbe(
            double probeAverageCurrentAmps,
            double probeAverageVelocityRps,
            double currentThresholdAmps,
            double velocityThresholdRps) {
        return probeAverageCurrentAmps >= currentThresholdAmps;
    }

    static boolean shouldAllowRelease(
            boolean canFire,
            boolean releaseStartedThisHold) {
        if (releaseStartedThisHold) {
            return true;
        }
        return canFire;
    }

    static boolean shouldTreatFlywheelAsReady(
            boolean flywheelAtSetpoint,
            boolean releaseStartedThisHold,
            boolean flywheelReadyLatchedThisHold) {
        if (!releaseStartedThisHold) {
            return flywheelAtSetpoint;
        }
        return flywheelReadyLatchedThisHold;
    }

    static double getAlignmentToleranceRadForShot(boolean shotIsFeedMode, double normalToleranceRad) {
        return shotIsFeedMode ? kFeedAimAlignmentToleranceRad : normalToleranceRad;
    }
}
