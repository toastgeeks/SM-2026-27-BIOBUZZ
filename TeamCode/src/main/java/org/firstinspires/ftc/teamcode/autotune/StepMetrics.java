package org.firstinspires.ftc.teamcode.autotune;

/**
 * Watches the response to one step in target velocity. Velocity is lightly smoothed because the
 * hub's velocity readout is quantized at low speed. A "flip" is the error changing sign by more
 * than the band, so one overshoot is 1 flip and a sustained oscillation keeps adding flips.
 */
class StepMetrics {
    private static final double SMOOTHING = 0.4; // weight of the newest sample

    private final double band;

    private double target;
    private double stepSize;
    private double direction;
    private double smoothed;
    private double peakOvershoot;
    private int flips;
    private int lastSign;

    StepMetrics(double band) {
        this.band = band;
    }

    void begin(double target, double startVelocity) {
        this.target = target;
        this.stepSize = Math.max(Math.abs(target - startVelocity), 1e-6);
        this.direction = Math.signum(target - startVelocity);
        this.smoothed = startVelocity;
        this.peakOvershoot = 0;
        this.flips = 0;
        this.lastSign = 0;
    }

    void add(double velocity) {
        smoothed += SMOOTHING * (velocity - smoothed);
        double error = smoothed - target;
        peakOvershoot = Math.max(peakOvershoot, direction * error);
        if (Math.abs(error) > band) {
            int sign = error > 0 ? 1 : -1;
            if (lastSign != 0 && sign != lastSign) flips++;
            lastSign = sign;
        }
    }

    /** The smoothed velocity, which is steadier than a single reading. */
    double smoothed() {
        return smoothed;
    }

    double stepSize() {
        return stepSize;
    }

    boolean isStepUp() {
        return direction > 0;
    }

    /** Smoothed velocity minus the target, so positive means above it. */
    double error() {
        return smoothed - target;
    }

    /** Peak overshoot in velocity units. */
    double peakOvershoot() {
        return peakOvershoot;
    }

    /** Peak overshoot as a fraction of the target speed. */
    double overshootOfTarget() {
        return target > 0 ? peakOvershoot / target : 0;
    }

    /** Peak overshoot as a fraction of the step size. */
    double overshoot() {
        return peakOvershoot / stepSize;
    }

    int flips() {
        return flips;
    }
}
