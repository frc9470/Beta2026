package com.team9470.telemetry;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.team9470.telemetry.PracticeTimerTracker.DriverStationSample;
import com.team9470.telemetry.PracticeTimerTracker.Phase;
import com.team9470.telemetry.PracticeTimerTracker.Zone;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.DriverStation.MatchType;
import java.util.Optional;
import org.junit.jupiter.api.Test;

class PracticeTimerTrackerTest {
    private static DriverStationSample sample(
            double nowSec,
            MatchType matchType,
            boolean fmsAttached,
            boolean autoEnabled,
            boolean teleopEnabled,
            boolean testEnabled,
            boolean disabled,
            double matchTimeSec) {
        return new DriverStationSample(
                nowSec,
                matchType,
                fmsAttached,
                true,
                autoEnabled,
                teleopEnabled,
                testEnabled,
                disabled,
                matchTimeSec,
                Optional.of(Alliance.Blue),
                "");
    }

    private static DriverStationSample sample(
            double nowSec,
            MatchType matchType,
            boolean autoEnabled,
            boolean teleopEnabled,
            boolean testEnabled,
            boolean disabled,
            double matchTimeSec) {
        return sample(nowSec, matchType, false, autoEnabled, teleopEnabled, testEnabled, disabled, matchTimeSec);
    }

    @Test
    void practiceMatchTypeEdgeTransitionsToArmed() {
        PracticeTimerTracker tracker = new PracticeTimerTracker();
        tracker.update(sample(0.00, MatchType.None, false, false, false, true, 0.0));

        var out = tracker.update(sample(0.02, MatchType.Practice, false, false, false, true, 0.0));

        assertEquals(Phase.ARMED.code(), out.snapshot().phaseCode());
        assertTrue(out.snapshot().active());
        assertEquals(1, out.snapshot().runId());
        assertEquals(PracticeTimerTracker.START_SOURCE_PRACTICE_EDGE, out.snapshot().startSourceCode());
    }

    @Test
    void armedAndAutoEnabledTransitionsToAuto() {
        PracticeTimerTracker tracker = new PracticeTimerTracker();
        tracker.update(sample(0.00, MatchType.Practice, false, false, false, true, 0.0));

        var out = tracker.update(sample(0.04, MatchType.Practice, true, false, false, false, 15.0));

        assertEquals(Phase.AUTO.code(), out.snapshot().phaseCode());
    }

    @Test
    void autoDisableTransitionsToTransition() {
        PracticeTimerTracker tracker = new PracticeTimerTracker();
        tracker.update(sample(0.00, MatchType.Practice, false, false, false, true, 0.0));
        tracker.update(sample(0.04, MatchType.Practice, true, false, false, false, 15.0));

        var out = tracker.update(sample(0.08, MatchType.Practice, false, false, false, true, 0.0));

        assertEquals(Phase.TRANSITION.code(), out.snapshot().phaseCode());
    }

    @Test
    void transitionTeleopEnableTransitionsToTeleop() {
        PracticeTimerTracker tracker = new PracticeTimerTracker();
        tracker.update(sample(0.00, MatchType.Practice, false, false, false, true, 0.0));
        tracker.update(sample(0.04, MatchType.Practice, true, false, false, false, 15.0));
        tracker.update(sample(0.08, MatchType.Practice, false, false, false, true, 0.0));

        var out = tracker.update(sample(0.12, MatchType.Practice, false, true, false, false, 135.0));

        assertEquals(Phase.TELEOP.code(), out.snapshot().phaseCode());
    }

    @Test
    void transitionDisabledDoesNotAbortBeforeTeleopEnable() {
        PracticeTimerTracker tracker = new PracticeTimerTracker();
        tracker.update(sample(0.00, MatchType.Practice, false, false, false, true, 0.0));
        tracker.update(sample(0.04, MatchType.Practice, true, false, false, false, 15.0));
        tracker.update(sample(0.08, MatchType.Practice, false, false, false, true, 0.0));

        var waiting = tracker.update(sample(3.00, MatchType.Practice, false, false, false, true, 0.0));
        var teleop = tracker.update(sample(3.04, MatchType.Practice, false, true, false, false, 135.0));

        assertEquals(Phase.TRANSITION.code(), waiting.snapshot().phaseCode());
        assertEquals(Phase.TELEOP.code(), teleop.snapshot().phaseCode());
    }

    @Test
    void teleopDisableTransitionsToEndedAndLatches() {
        PracticeTimerTracker tracker = new PracticeTimerTracker();
        tracker.update(sample(0.00, MatchType.Practice, false, false, false, true, 0.0));
        tracker.update(sample(0.04, MatchType.Practice, true, false, false, false, 15.0));
        tracker.update(sample(0.08, MatchType.Practice, false, false, false, true, 0.0));
        tracker.update(sample(0.12, MatchType.Practice, false, true, false, false, 135.0));

        var end = tracker.update(sample(1.00, MatchType.Practice, false, false, false, true, 0.0));
        var latched = tracker.update(sample(2.00, MatchType.Practice, false, false, false, true, 0.0));

        assertEquals(Phase.ENDED.code(), end.snapshot().phaseCode());
        assertTrue(end.snapshot().complete());
        assertEquals(Phase.ENDED.code(), latched.snapshot().phaseCode());
    }

    @Test
    void autoEnableOutsidePracticeAndFmsDoesNotStartRun() {
        PracticeTimerTracker tracker = new PracticeTimerTracker();
        tracker.update(sample(0.00, MatchType.None, false, false, false, true, 0.0));

        var out = tracker.update(sample(0.02, MatchType.None, true, false, false, false, 15.0));

        assertEquals(Phase.IDLE.code(), out.snapshot().phaseCode());
        assertEquals(0, out.snapshot().runId());
        assertFalse(out.snapshot().active());
    }

    @Test
    void teleopEnableOutsidePracticeDoesNotStartRun() {
        PracticeTimerTracker tracker = new PracticeTimerTracker();

        var out = tracker.update(sample(0.02, MatchType.None, false, true, false, false, 0.0));

        assertEquals(Phase.IDLE.code(), out.snapshot().phaseCode());
        assertEquals(0, out.snapshot().runId());
        assertFalse(out.snapshot().active());
    }

    @Test
    void teleopEnableOnFmsStartsRun() {
        PracticeTimerTracker tracker = new PracticeTimerTracker();
        tracker.update(sample(0.00, MatchType.None, true, false, false, false, true, 0.0));

        var out = tracker.update(sample(0.02, MatchType.None, true, false, true, false, false, 120.0));

        assertEquals(Phase.TELEOP.code(), out.snapshot().phaseCode());
        assertEquals(PracticeTimerTracker.START_SOURCE_AUTO_EDGE_FALLBACK, out.snapshot().startSourceCode());
        assertTrue(out.snapshot().active());
    }

    @Test
    void autoEnableOnFmsStartsRun() {
        PracticeTimerTracker tracker = new PracticeTimerTracker();
        tracker.update(sample(0.00, MatchType.None, true, false, false, false, true, 0.0));

        var out = tracker.update(sample(0.02, MatchType.None, true, true, false, false, false, 15.0));

        assertEquals(Phase.AUTO.code(), out.snapshot().phaseCode());
        assertEquals(PracticeTimerTracker.START_SOURCE_AUTO_EDGE_FALLBACK, out.snapshot().startSourceCode());
        assertTrue(out.snapshot().active());
    }

    @Test
    void invalidMatchTimeUsesLocalFallbackAndRemainsMonotonic() {
        PracticeTimerTracker tracker = new PracticeTimerTracker();
        tracker.update(sample(0.00, MatchType.Practice, false, false, false, true, 0.0));

        var first = tracker.update(sample(0.04, MatchType.Practice, true, false, false, false, Double.NaN));
        var second = tracker.update(sample(0.08, MatchType.Practice, true, false, false, false, Double.NaN));

        assertEquals(Phase.AUTO.code(), first.snapshot().phaseCode());
        assertFalse(first.snapshot().usingDsMatchTime());
        assertFalse(second.snapshot().usingDsMatchTime());
        assertTrue(second.snapshot().phaseRemainingSec() <= first.snapshot().phaseRemainingSec());
    }

    @Test
    void newRunIncrementsRunId() {
        PracticeTimerTracker tracker = new PracticeTimerTracker();
        tracker.update(sample(0.00, MatchType.Practice, false, false, false, true, 0.0));
        tracker.update(sample(0.04, MatchType.Practice, true, false, false, false, 15.0));
        tracker.update(sample(0.08, MatchType.Practice, false, false, false, true, 0.0));
        tracker.update(sample(0.12, MatchType.Practice, false, true, false, false, 135.0));
        tracker.update(sample(1.00, MatchType.Practice, false, false, false, true, 0.0));

        tracker.update(sample(1.04, MatchType.None, false, false, false, true, 0.0));
        var secondRun = tracker.update(sample(1.08, MatchType.Practice, false, false, false, true, 0.0));

        assertEquals(2, secondRun.snapshot().runId());
        assertEquals(Phase.ARMED.code(), secondRun.snapshot().phaseCode());
    }

    @Test
    void endgameIsLastThirtySecondsOfTeleop() {
        PracticeTimerTracker tracker = new PracticeTimerTracker();
        tracker.update(sample(0.00, MatchType.Practice, false, false, false, true, 0.0));
        tracker.update(sample(0.04, MatchType.Practice, true, false, false, false, 20.0));
        tracker.update(sample(0.08, MatchType.Practice, false, false, false, true, 0.0));
        tracker.update(sample(0.12, MatchType.Practice, false, true, false, false, 140.0));

        var beforeEndgame = tracker.update(sample(1.00, MatchType.Practice, false, true, false, false, 31.0));
        var atEndgame = tracker.update(sample(1.04, MatchType.Practice, false, true, false, false, 30.0));

        assertEquals(Zone.SHIFT4.code(), beforeEndgame.snapshot().zoneCode());
        assertFalse(beforeEndgame.snapshot().endgame());
        assertEquals(Zone.ENDGAME.code(), atEndgame.snapshot().zoneCode());
        assertTrue(atEndgame.snapshot().endgame());
    }

    @Test
    void transitionZoneIsTenSecondsThenShiftOneStarts() {
        PracticeTimerTracker tracker = new PracticeTimerTracker();
        tracker.update(sample(0.00, MatchType.Practice, false, false, false, true, 0.0));
        tracker.update(sample(0.04, MatchType.Practice, true, false, false, false, 20.0));
        tracker.update(sample(0.08, MatchType.Practice, false, false, false, true, 0.0));
        tracker.update(sample(0.12, MatchType.Practice, false, true, false, false, 140.0));

        var stillTransition = tracker.update(sample(0.50, MatchType.Practice, false, true, false, false, 130.1));
        var shiftOne = tracker.update(sample(0.54, MatchType.Practice, false, true, false, false, 130.0));

        assertEquals(Zone.TRANSITION.code(), stillTransition.snapshot().zoneCode());
        assertEquals(Zone.SHIFT1.code(), shiftOne.snapshot().zoneCode());
    }
}
