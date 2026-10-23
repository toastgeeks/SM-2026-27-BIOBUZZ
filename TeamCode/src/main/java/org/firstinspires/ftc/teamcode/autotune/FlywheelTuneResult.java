package org.firstinspires.ftc.teamcode.autotune;

import java.util.Locale;

/** Coefficients found by {@link FlywheelAutoTuner}, plus how well they performed. */
public class FlywheelTuneResult {
    public final double p, i, d, f;
    public final double voltage;
    /** Mean absolute error over the validation holds, as a fraction of target. */
    public final double meanError;
    /** Mean absolute error while following the ramps, as a fraction of max velocity. */
    public final double rampError;
    /** Seconds the spin-up from idle took to reach the rise fraction (75%) of the top speed, or -1 if it never did. */
    public final double spinUpSeconds;
    /** Largest overshoot of the target during a validation spin-up or step, as a fraction of the target. */
    public final double overshoot;
    /** Seconds the spin-up from idle took to get within the settle band (2%) of the target, or -1 if it never did. */
    public final double settleSeconds;
    public final boolean passed;
    /** The encoder reads opposite to the power, so only F was tuned and P is 0. */
    public final boolean encoderInverted;
    public final String notes;
    /** Direction each motor ended up with, e.g. "shooter: FORWARD, shooter2: REVERSE". */
    public final String directions;
    /** Error at each validation hold, e.g. "30%:+1.2%  90%:-4.0%" (target as a share of max : error). */
    public final String holdReport;
    /** How the legs ended and how noisy the speed readings were. */
    public final String diagnostics;
    /** Every value the searches tried, and whether it went over. */
    public final String searchReport;

    FlywheelTuneResult(double p, double i, double d, double f, double voltage,
               double meanError, double rampError, double spinUpSeconds, double settleSeconds, double overshoot, boolean passed, boolean encoderInverted, String notes, String directions,
               String holdReport, String diagnostics, String searchReport) {
        this.p = p;
        this.i = i;
        this.d = d;
        this.f = f;
        this.voltage = voltage;
        this.meanError = meanError;
        this.rampError = rampError;
        this.spinUpSeconds = spinUpSeconds;
        this.settleSeconds = settleSeconds;
        this.overshoot = overshoot;
        this.encoderInverted = encoderInverted;
        this.passed = passed;
        this.notes = notes;
        this.directions = directions;
        this.holdReport = holdReport;
        this.diagnostics = diagnostics;
        this.searchReport = searchReport;
    }

    /** A line you can paste straight into code. */
    public String asConstants() {
        return String.format(Locale.US, "P=%.4f  I=%.4f  D=%.4f  F=%.4f", p, i, d, f);
    }
}
