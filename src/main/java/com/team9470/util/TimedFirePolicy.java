package com.team9470.util;

public final class TimedFirePolicy {
    public static final int REASON_NOT_REQUESTED = 0;
    public static final int REASON_NORMAL_IMMEDIATE = 1;
    public static final int REASON_WAITING_FOR_WINDOW = 2;
    public static final int REASON_TIMING_UNKNOWN = 3;
    public static final int REASON_TOP_SENSOR_CLEAR = 4;
    public static final int REASON_SHOOTER_NOT_READY = 5;
    public static final int REASON_NOT_ALIGNED = 6;
    public static final int REASON_SHOT_INVALID = 7;
    public static final int REASON_FEED_MODE = 8;
    public static final int REASON_RELEASE_ALLOWED = 9;

    private static final double kFeederTransitSec = 0.08;
    private static final double kHubProcessingDelaySec = 0.10;
    private static final double kReleaseMarginSec = 0.03;

    private TimedFirePolicy() {
    }

    public static TimedFireDecision evaluate(
            boolean driverWantsShoot,
            boolean timingKnown,
            boolean zoneActive,
            double zoneRemainingSec,
            boolean buttonPressedDuringInactiveThisHold,
            boolean topBeamBreakBlocked,
            boolean shooterAtSetpoint,
            boolean aligned,
            boolean shotIsValid,
            boolean shotIsFeedMode,
            AutoAim.ShootingSolution solution) {
        double naiveAirTimeSec = solution == null ? 0.0 : Math.max(0.0, solution.naiveAirTimeSec());
        double launchLeadSec = naiveAirTimeSec + kFeederTransitSec + kHubProcessingDelaySec + kReleaseMarginSec;
        boolean releaseWindowOpen = timingKnown && (zoneActive || zoneRemainingSec <= launchLeadSec);

        if (!driverWantsShoot) {
            return new TimedFireDecision(false, false, false, false, releaseWindowOpen, launchLeadSec, REASON_NOT_REQUESTED);
        }
        if (shotIsFeedMode) {
            return new TimedFireDecision(
                    false,
                    true,
                    false,
                    buttonPressedDuringInactiveThisHold,
                    releaseWindowOpen,
                    launchLeadSec,
                    REASON_FEED_MODE);
        }
        if (!buttonPressedDuringInactiveThisHold) {
            return new TimedFireDecision(false, true, false, false, releaseWindowOpen, launchLeadSec, REASON_NORMAL_IMMEDIATE);
        }
        if (!timingKnown) {
            return new TimedFireDecision(true, true, false, true, false, launchLeadSec, REASON_TIMING_UNKNOWN);
        }
        if (!shotIsValid) {
            return new TimedFireDecision(true, true, false, true, releaseWindowOpen, launchLeadSec, REASON_SHOT_INVALID);
        }
        if (!topBeamBreakBlocked) {
            return new TimedFireDecision(true, true, false, true, releaseWindowOpen, launchLeadSec, REASON_TOP_SENSOR_CLEAR);
        }
        if (!releaseWindowOpen) {
            return new TimedFireDecision(true, true, false, true, false, launchLeadSec, REASON_WAITING_FOR_WINDOW);
        }
        if (!shooterAtSetpoint) {
            return new TimedFireDecision(true, true, false, true, releaseWindowOpen, launchLeadSec, REASON_SHOOTER_NOT_READY);
        }
        if (!aligned) {
            return new TimedFireDecision(true, true, false, true, releaseWindowOpen, launchLeadSec, REASON_NOT_ALIGNED);
        }
        return new TimedFireDecision(true, true, true, true, releaseWindowOpen, launchLeadSec, REASON_RELEASE_ALLOWED);
    }

    public static String reasonLabel(int reasonCode) {
        return switch (reasonCode) {
            case REASON_NOT_REQUESTED -> "NotRequested";
            case REASON_NORMAL_IMMEDIATE -> "NormalImmediate";
            case REASON_WAITING_FOR_WINDOW -> "WaitingForWindow";
            case REASON_TIMING_UNKNOWN -> "TimingUnknown";
            case REASON_TOP_SENSOR_CLEAR -> "TopSensorClear";
            case REASON_SHOOTER_NOT_READY -> "ShooterNotReady";
            case REASON_NOT_ALIGNED -> "NotAligned";
            case REASON_SHOT_INVALID -> "ShotInvalid";
            case REASON_FEED_MODE -> "FeedMode";
            case REASON_RELEASE_ALLOWED -> "ReleaseAllowed";
            default -> "Unknown";
        };
    }

    public record TimedFireDecision(
            boolean timedAutoArmCandidate,
            boolean spinShooter,
            boolean allowFeed,
            boolean armedThisHold,
            boolean releaseWindowOpen,
            double launchLeadSec,
            int reasonCode) {
    }
}
