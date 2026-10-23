package org.firstinspires.ftc.teamcode.autotune;

import java.util.ArrayDeque;

/**
 * The last few seconds of velocity samples. A leg is steady when the mean of the newer half
 * matches the mean of the older half, which tolerates the quantization noise in the hub's
 * velocity readout.
 */
class SteadyWindow {
    private final double span;
    private final ArrayDeque<double[]> samples = new ArrayDeque<>();

    SteadyWindow(double spanSeconds) {
        this.span = spanSeconds;
    }

    void reset() {
        samples.clear();
    }

    void add(double now, double velocity) {
        samples.addLast(new double[]{now, velocity});
        while (now - samples.peekFirst()[0] > span) samples.removeFirst();
    }

    /** True once the window covers nearly its whole span. */
    boolean isFull() {
        return samples.size() > 1 && samples.peekLast()[0] - samples.peekFirst()[0] >= 0.8 * span;
    }

    double mean() {
        double sum = 0;
        for (double[] s : samples) sum += s[1];
        return samples.isEmpty() ? 0 : sum / samples.size();
    }

    /**
     * How much the newer-half mean can differ from the older-half mean by chance alone. The noise is taken
     * from sample-to-sample differences, so a flywheel that is still climbing does not look noisy.
     */
    double noiseBand() {
        int n = samples.size();
        if (n < 4) return Double.MAX_VALUE;
        return 3 * noise() * 2 / Math.sqrt(n);
    }

    /** Typical size of the sample-to-sample jitter, in the same units as the samples. */
    double noise() {
        int n = samples.size();
        if (n < 4) return 0;
        double sumSq = 0, previous = 0;
        boolean first = true;
        for (double[] s : samples) {
            if (!first) sumSq += (s[1] - previous) * (s[1] - previous);
            previous = s[1];
            first = false;
        }
        return Math.sqrt(sumSq / (2.0 * (n - 1)));
    }

    /** Newer-half mean minus older-half mean. */
    double drift() {
        double mid = (samples.peekFirst()[0] + samples.peekLast()[0]) / 2;
        double oldSum = 0, newSum = 0;
        int oldCount = 0, newCount = 0;
        for (double[] s : samples) {
            if (s[0] < mid) {
                oldSum += s[1];
                oldCount++;
            } else {
                newSum += s[1];
                newCount++;
            }
        }
        return (newCount == 0 || oldCount == 0) ? 0 : newSum / newCount - oldSum / oldCount;
    }
}
