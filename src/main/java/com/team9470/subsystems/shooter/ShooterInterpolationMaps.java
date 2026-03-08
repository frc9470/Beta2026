package com.team9470.subsystems.shooter;

import java.util.Optional;

import com.team254.lib.util.InterpolatingDouble;
import com.team254.lib.util.InterpolatingTreeMap;

public final class ShooterInterpolationMaps {
    private ShooterInterpolationMaps() {
    }

    private static final double kFeedLaunchAngleMultiplier = 1.4;
    private static final InterpolatingTreeMap<InterpolatingDouble, ShotParameter> kHubMap = new InterpolatingTreeMap<>();
    private static final InterpolatingTreeMap<InterpolatingDouble, ShotParameter> kFeedMap = new InterpolatingTreeMap<>();

    // Air-time (time-of-flight) map: distance (m) -> seconds.
    // Adapted from Team Ninja's measured data; tune on-robot as needed.
    private static final InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> kAirTimeMap = new InterpolatingTreeMap<>();

    static {
        loadHubMap();
        loadFeedMap();
        loadAirTimeMap();
    }

    private static void loadHubMap() {
        addHubPoint(1.0, 15.0, 1850.0); // center doesn't work
        addHubPoint(1.25, 18.0, 1750.0);
        addHubPoint(1.5, 17.0, 1800.0);
        addHubPoint(1.75, 17.0, 1900.0);
        addHubPoint(2.0, 20.0, 1950.0);
        addHubPoint(3.0, 31.0, 2070.0);
        addHubPoint(3.5, 33.0, 2190.0);
        addHubPoint(4.0, 34.0, 2300.0);
        addHubPoint(4.5, 34.0, 2530.0);
        addHubPoint(5.0, 37.0, 2850.0);
    }

    private static void loadFeedMap() {
        // Feed map starts from hub-like points and can be tuned independently.
        addFeedPoint(1.0, 15.0, 1850.0);
        addFeedPoint(1.25, 18.0, 1750.0);
        addFeedPoint(1.5, 17.0, 1800.0);
        addFeedPoint(1.75, 17.0, 1900.0);
        addFeedPoint(2.0, 20.0, 1950.0);
        addFeedPoint(3.0, 31.0, 2070.0);
        addFeedPoint(3.5, 33.0, 2190.0);
        addFeedPoint(4.0, 34.0, 2300.0);
        addFeedPoint(4.5, 34.0, 2530.0);
        addFeedPoint(5.0, 37.0, 2800.0);
        addFeedPoint(6.0, 39.0, 3000.0);
    }

    private static void loadAirTimeMap() {
        // Air-time data (distance m -> seconds)
        addAirTimePoint(0.0, 0.0);
        addAirTimePoint(1.5, .71);
        addAirTimePoint(2.0, .83);
        addAirTimePoint(2.5, 1.12);
        addAirTimePoint(3.0, 1.03);
        addAirTimePoint(3.5, 1.01);
        addAirTimePoint(4.0, 1.14);
        addAirTimePoint(4.5, 1.38);
        addAirTimePoint(5.0, 1.30);
    }

    /**
     * Returns the estimated projectile time-of-flight for a given distance.
     */
    public static double getAirTime(double distanceMeters) {
        InterpolatingDouble result = kAirTimeMap.getInterpolated(interpolatingDouble(distanceMeters));
        return result != null ? result.value : 0.0;
    }

    public static Optional<ShotParameter> getHub(double distanceMeters) {
        ShotParameter parameter = kHubMap.getInterpolated(interpolatingDouble(distanceMeters));
        return Optional.ofNullable(parameter);
    }

    public static void addHubPoint(double distanceMeters, double hoodCommandDeg, double flywheelRpm) {
        kHubMap.put(interpolatingDouble(distanceMeters), new ShotParameter(hoodCommandDeg, flywheelRpm));
    }

    public static Optional<ShotParameter> getFeed(double distanceMeters) {
        ShotParameter parameter = kFeedMap.getInterpolated(interpolatingDouble(distanceMeters));
        if (parameter == null) {
            return Optional.empty();
        }
        return Optional.of(new ShotParameter(
                parameter.hoodCommandDeg() * kFeedLaunchAngleMultiplier,
                parameter.flywheelRpm()));
    }

    public static void addFeedPoint(double distanceMeters, double hoodCommandDeg, double flywheelRpm) {
        kFeedMap.put(interpolatingDouble(distanceMeters), new ShotParameter(hoodCommandDeg, flywheelRpm));
    }

    private static void addAirTimePoint(double distanceMeters, double airTimeSec) {
        kAirTimeMap.put(interpolatingDouble(distanceMeters), interpolatingDouble(airTimeSec));
    }

    private static InterpolatingDouble interpolatingDouble(double value) {
        return new InterpolatingDouble(value);
    }
}
