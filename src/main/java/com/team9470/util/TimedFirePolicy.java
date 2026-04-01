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

        if (!driverWantsShoot) {
            return new TimedFireDecision(false, false, false, false, launchLeadSec, REASON_NOT_REQUESTED);
        }
        if (shotIsFeedMode) {
            return new TimedFireDecision(false, true, false, buttonPressedDuringInactiveThisHold, launchLeadSec, REASON_FEED_MODE);
        }
        if (!buttonPressedDuringInactiveThisHold) {
            return new TimedFireDecision(false, true, false, false, launchLeadSec, REASON_NORMAL_IMMEDIATE);
        }
        if (!timingKnown) {
            return new TimedFireDecision(true, true, false, true, launchLeadSec, REASON_TIMING_UNKNOWN);
        }
        if (!shotIsValid) {
            return new TimedFireDecision(true, true, false, true, launchLeadSec, REASON_SHOT_INVALID);
        }
        if (!topBeamBreakBlocked) {
            return new TimedFireDecision(true, true, false, true, launchLeadSec, REASON_TOP_SENSOR_CLEAR);
        }

        boolean releaseWindowOpen = zoneActive || zoneRemainingSec <= launchLeadSec;
        if (!releaseWindowOpen) {
            return new TimedFireDecision(true, true, false, true, launchLeadSec, REASON_WAITING_FOR_WINDOW);
        }
        if (!shooterAtSetpoint) {
            return new TimedFireDecision(true, true, false, true, launchLeadSec, REASON_SHOOTER_NOT_READY);
        }
        if (!aligned) {
            return new TimedFireDecision(true, true, false, true, launchLeadSec, REASON_NOT_ALIGNED);
        }
        return new TimedFireDecision(true, true, true, true, launchLeadSec, REASON_RELEASE_ALLOWED);
    }

    public record TimedFireDecision(
            boolean timedAutoArmCandidate,
            boolean spinShooter,
            boolean allowFeed,
            boolean armedThisHold,
            double launchLeadSec,
            int reasonCode) {
    }
}
