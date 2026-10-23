package org.firstinspires.ftc.teamcode.autotune;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

/**
 * The target-velocity path the tuner drives the flywheel along, as fractions of max velocity.
 * Hold legs end when the flywheel settles (or times out) rather than after a fixed time, so a
 * slow flywheel gets the seconds it needs and a fast one does not wait. Random targets use a
 * fixed seed so every run, and every cycle, sees the same path.
 */
final class TargetProfile {
    static final class Leg {
        final boolean ramp;
        final boolean scored;   // false for a warm-up hold that only gets the flywheel to its starting speed
        final double from, to, seconds;

        private Leg(boolean ramp, boolean scored, double from, double to, double seconds) {
            this.ramp = ramp;
            this.scored = scored;
            this.from = from;
            this.to = to;
            this.seconds = seconds;
        }

        static Leg hold(double fraction) {
            return new Leg(false, true, fraction, fraction, 0);
        }

        static Leg warmup(double fraction) {
            return new Leg(false, false, fraction, fraction, 0);
        }

        static Leg ramp(double from, double to, double seconds) {
            return new Leg(true, true, from, to, seconds);
        }

        double fractionAt(double t) {
            return ramp ? from + (to - from) * Math.min(t / seconds, 1) : to;
        }
    }

    private TargetProfile() {}

    /** Targets in [lo, hi], each at least minGap away from the one before. */
    static double[] randomTargets(int count, double lo, double hi, double minGap, long seed) {
        Random random = new Random(seed);
        // A gap bigger than the range allows would leave no valid next target
        minGap = Math.min(minGap, 0.4 * (hi - lo));
        double[] targets = new double[count];
        for (int i = 0; i < count; i++) {
            double next;
            do {
                next = lo + random.nextDouble() * (hi - lo);
            } while (i > 0 && Math.abs(next - targets[i - 1]) < minGap);
            targets[i] = next;
        }
        return targets;
    }

    /**
     * Idle (not scored), a spin-up from idle to the centre speed (timed, scored), a ramp to the random-low speed, a hold
     * there, a ramp up to the high speed, a hold there, then random holds between the random-low and high speeds.
     * Everything is a fraction of the top speed: the band is random-low (75%) up to high (95%).
     */
    static List<Leg> validation(FlywheelAutoTuner.Config cfg) {
        List<Leg> legs = new ArrayList<>();
        legs.add(Leg.warmup(cfg.profileLowFraction));
        legs.add(Leg.hold(cfg.profileCentreFraction));
        // When the spin-up already goes to the bottom of the band there is nothing to ramp down to
        if (Math.abs(cfg.profileCentreFraction - cfg.randomLowFraction) > 0.02) {
            legs.add(Leg.ramp(cfg.profileCentreFraction, cfg.randomLowFraction, cfg.rampDownSeconds));
            legs.add(Leg.hold(cfg.randomLowFraction));
        }
        legs.add(Leg.ramp(cfg.randomLowFraction, cfg.profileHighFraction, cfg.rampUpSeconds));
        legs.add(Leg.hold(cfg.profileHighFraction));
        for (double target : randomTargets(cfg.validateRandomTargets, cfg.randomLowFraction,
                cfg.profileHighFraction, cfg.minStepGapFraction, cfg.profileSeed + 1)) {
            legs.add(Leg.hold(target));
        }
        return legs;
    }
}
