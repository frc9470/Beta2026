package com.team9470.subsystems;

import com.team9470.TunerConstants;

/**
 * Constants used by the superstructure coordinator.
 *
 * <p>
 * Mechanism-specific constants still live with their owning subsystems. These
 * values are for coordination, aiming, and telemetry state owned by
 * {@link Superstructure}.
 */
public final class SuperstructureConstants {
    private SuperstructureConstants() {
    }

    public static final class Aim {
        public static final double kAimKpDefault = 12.0;
        public static final double kAimKiDefault = 0.0;
        public static final double kAimKdDefault = 1.0;
        public static final double kAimAlignmentToleranceRadDefault = Math.toRadians(2.5);
        public static final double kFeedAimAlignmentToleranceRad = Math.toRadians(10.0);
        public static final double kAimMaxIntegralContributionRadPerSecDefault = Math.toRadians(25.0);
        public static final double kAimMaxAngularRateRadPerSec = Math.toRadians(TunerConstants.maxAngularVelocity);

        private Aim() {
        }
    }

    public static final class ActiveIdleBoost {
        /** Seconds before / after the active zone to keep the flywheel boosted. */
        public static final double kIdleBoostPaddingSec = 3.0;

        private ActiveIdleBoost() {
        }
    }

    public static final class DriveAssist {
        // COR shifting constants (ported from MechAdv DriveCommands)
        public static final double kCORMinErrorDeg = 15.0;
        public static final double kCORMaxErrorDeg = 30.0;

        // O-Lock thresholds (ported from MechAdv DriveCommands)
        public static final double kOLockSpeedThresholdMps = 0.1;
        public static final double kOLockOmegaThresholdRadPerSec = 0.15;

        private DriveAssist() {
        }
    }

    public static final class ManualIntake {
        /** Threshold in seconds: presses shorter than this are treated as a tap. */
        public static final double kManualIntakeTapThresholdSec = 0.30;

        private ManualIntake() {
        }
    }

    public static final class Preload {
        public static final double kPreloadSettleVolts = -7.0;
        public static final double kPreloadSettleSec = 0.50;
        public static final double kPreloadStageHopperVolts = -3.5;
        public static final double kPreloadStageFeederVolts = -1.5;
        public static final double kPreloadTimeoutSec = 2.5;
        public static final double kPreloadJamCurrentAmps = 18.0;
        public static final double kPreloadJamVelocityRps = 1.0;
        public static final double kPreloadJamDebounceSec = 0.20;

        public static final int PRELOAD_PHASE_IDLE = 0;
        public static final int PRELOAD_PHASE_ALREADY_STAGED = 1;
        public static final int PRELOAD_PHASE_SETTLING = 2;
        public static final int PRELOAD_PHASE_STAGING = 3;
        public static final int PRELOAD_PHASE_DONE = 4;

        public static final int PRELOAD_FAULT_NONE = 0;
        public static final int PRELOAD_FAULT_TIMEOUT = 1;
        public static final int PRELOAD_FAULT_JAM = 2;

        private Preload() {
        }
    }

    public static final class AutoStage {
        public static final String kAutoStageEnabledKey = "Debug/HopperAutoStage/Enabled";
        public static final String kAutoStageProbeVoltsKey = "Debug/HopperAutoStage/ProbeVolts";
        public static final String kAutoStageProbeBaselineSecKey = "Debug/HopperAutoStage/ProbeBaselineSec";
        public static final String kAutoStageProbePulseSecKey = "Debug/HopperAutoStage/ProbePulseSec";
        public static final String kAutoStageProbeCurrentThresholdKey =
                "Debug/HopperAutoStage/ProbeCurrentThresholdAmps";
        public static final String kAutoStageProbeVelocityThresholdKey =
                "Debug/HopperAutoStage/ProbeVelocityThresholdRps";
        public static final String kAutoStageIntakeCurrentThresholdKey =
                "Debug/HopperAutoStage/IntakeCurrentThresholdAmps";
        public static final String kAutoStageLoadScoreThresholdKey = "Debug/HopperAutoStage/LoadScoreThresholdSec";
        public static final String kAutoStageLoadScoreDecayKey = "Debug/HopperAutoStage/LoadScoreDecayPerSec";
        public static final String kAutoStageQuietTimeSecKey = "Debug/HopperAutoStage/QuietTimeSec";
        public static final String kAutoStageCooldownSecKey = "Debug/HopperAutoStage/CooldownSec";

        public static final boolean kAutoStageEnabledDefault = true;
        public static final double kAutoStageProbeVoltsDefault = -2.5;
        public static final double kAutoStageProbeBaselineSecDefault = 0.08;
        public static final double kAutoStageProbePulseSecDefault = 0.20;
        public static final double kAutoStageProbeCurrentThresholdAmpsDefault = 30.0;
        public static final double kAutoStageProbeVelocityThresholdRpsDefault = 4.0;
        public static final double kAutoStageIntakeCurrentThresholdAmpsDefault = 18.0;
        public static final double kAutoStageLoadScoreThresholdSecDefault = 0.35;
        public static final double kAutoStageLoadScoreDecayPerSecDefault = 0.20;
        public static final double kAutoStageQuietTimeSecDefault = 0.25;
        public static final double kAutoStageCooldownSecDefault = 1.50;
        public static final double kAutoStageLoadScoreMaxSec = 3.0;

        public static final int AUTO_STAGE_PHASE_IDLE = 0;
        public static final int AUTO_STAGE_PHASE_READY = 1;
        public static final int AUTO_STAGE_PHASE_PROBE_BASELINE = 2;
        public static final int AUTO_STAGE_PHASE_PROBE_PULSE = 3;
        public static final int AUTO_STAGE_PHASE_STAGE_COMMAND = 4;
        public static final int AUTO_STAGE_PHASE_COMPLETE = 5;

        public static final int AUTO_STAGE_REASON_NONE = 0;
        public static final int AUTO_STAGE_REASON_DISABLED = 1;
        public static final int AUTO_STAGE_REASON_NOT_TELEOP = 2;
        public static final int AUTO_STAGE_REASON_SUPERSTRUCTURE_BUSY = 3;
        public static final int AUTO_STAGE_REASON_HOPPER_BUSY = 4;
        public static final int AUTO_STAGE_REASON_SHOOTER_FIRING = 5;
        public static final int AUTO_STAGE_REASON_AGITATING = 6;
        public static final int AUTO_STAGE_REASON_TOP_BLOCKED = 7;
        public static final int AUTO_STAGE_REASON_EVIDENCE_LOW = 8;
        public static final int AUTO_STAGE_REASON_WAITING_FOR_QUIET = 9;
        public static final int AUTO_STAGE_REASON_COOLDOWN = 10;
        public static final int AUTO_STAGE_REASON_PROBE_RUNNING = 11;
        public static final int AUTO_STAGE_REASON_PROBE_REJECTED = 12;
        public static final int AUTO_STAGE_REASON_STAGE_REQUESTED = 13;

        private AutoStage() {
        }
    }

    public static final class ShotSafety {
        public static final double kFeedFlywheelSetpointToleranceRps = 150.0 / 60.0;
        public static final double kShootMaxPolarVelocityRadPerSec = 0.5;
        public static final double kShootVelocityLimitMinRequestMps = 0.15;
        public static final double kSotmFireSafetyMinSpeedMps = 0.35;
        public static final double kSotmMaxAccelerationForFireMps2 = 3.5;
        public static final double kSotmMaxOmegaForFireRadPerSec = Math.toRadians(220.0);

        private ShotSafety() {
        }
    }
}
