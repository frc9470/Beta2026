# TODO

## High Priority
- [ ] Autonomous reliability (Owner: AADIT)
  - [ ] Make current `Choreo` routines repeatable on-robot.
  - [ ] Define autonomous goals:
    - [ ] Target number of pieces.
    - [ ] Target cycle time / scoring window.
  - [ ] Expose maintained routines in the chooser or delete dead routines.
- [ ] Shooting / auto-aim polish (Owner: PRANESH)
  - [ ] Finalize shot parameter tuning.
  - [ ] Validate moving-shot fire gating and feed-mode behavior on robot.
- [ ] Feeder preload / staging
  - [x] Add top-of-feeder note detection (`beam break` on `DIO 0`) so notes can be staged below the shooter.
  - [x] Implement explicit driver-triggered preload on `D-pad Up`.
  - [x] Implement a two-step preload routine:
    - [x] Run hopper long enough for notes to settle and align.
    - [x] Run hopper + feeder until the top sensor is blocked.
  - [x] Add preload safety behavior:
    - [x] Timeout / jam detect if the top sensor never trips.
    - [x] Stop at the top sensor before entering the shooter.
    - [x] Expose beam-break + preload diagnostic telemetry.
  - [ ] Tune settle / stage voltages and timeout on robot.
  - [ ] Decide whether any automatic preload trigger is still wanted:
    - [ ] After sustained intake / hopper fill.
    - [ ] When leaving an active scoring phase.
- [ ] Phase-timed shooter priming
  - [x] Use `PracticeTimerTracker` / `MatchTimingService` active/inactive timing to schedule early release on inactive -> active transitions.
  - [x] Start flywheel rev-up immediately when the aligned shoot hold begins during inactive time.
  - [x] Use the existing TOF map plus constant feeder / hub delay offsets.
  - [x] Keep feed manual/normal and overlay timed auto-arm only onto aligned shoot holds.
  - [ ] Tune the timing budget on robot:
    - [ ] Hood settle / aim settle time.
    - [ ] Feeder transit time to launch.
    - [ ] Ball flight time to hub.
    - [ ] Hub processing / scoring delay offset.
  - [ ] Validate or extend endgame window behavior.
  - [ ] Add field-test logging for commanded fire time vs observed score time.
- [ ] Simulation + logging (Owner: TYCHO)
  - [ ] Add autonomous simulation/regression coverage for at least one real routine.
  - [ ] Decide whether to adopt `AdvantageKit` or keep the current CTRE/NT logging path.
- [ ] Vision / pose estimation
  - [ ] Add odometry back into the separate pose path in `Swerve.getTxTyPose()`.
  - [ ] Verify tag-pose blending and stale-data thresholds on robot.
- [ ] Swerve tuning
  - [ ] Tune drive and steer response on carpet.
  - [ ] Tune heading lock and auto-aim rotation response.
  - [ ] Tune `Choreo` path-following PID for repeatable autos.

## Medium Priority
- [ ] Intake parity
  - [ ] Check note detection from both rollers.
  - [ ] Publish both left and right roller telemetry.
  - [ ] Simulate both rollers.
- [ ] Telemetry improvements
  - [ ] Publish key auto/shot metrics to dashboard.
  - [ ] Add alerts for flywheel not-at-speed and heading error.
- [ ] Auto routine cleanup
  - [ ] Standardize chooser names vs trajectory names.
  - [ ] Remove prototype / dead routines that are no longer field candidates.

## Low Priority
- [ ] Shoot on the move.
- [ ] Codebase cleanup
  - [ ] Remove stale TODOs/comments in drive + shooter code.
  - [ ] Fix or remove stale Team 254 TODOs/helpers that are no longer used.
  - [ ] Consolidate constants naming for auto and shot tuning.
- [ ] Documentation
  - [ ] Add/update setup notes for simulation and logging workflow.
  - [ ] Document autonomous routine naming / chooser expectations.

## Completed / Already In Repo
- [x] Standardized on `Choreo` for autonomous paths.
- [x] Button-driven aim-and-shoot commands are wired in `RobotContainer`.
- [x] Baseline auto routines already exist in `Autos.java`.
- [x] Basic shooter, intake, and vision simulation scaffolding exists.
- [x] Shooter default idle speed updated to 2000 RPM.
- [x] Beam-break preload staging scaffolding is implemented.
- [x] Inactive-window timed shot scaffolding is implemented for aligned shoot holds.
