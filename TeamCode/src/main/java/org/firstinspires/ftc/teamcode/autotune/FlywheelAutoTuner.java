package org.firstinspires.ftc.teamcode.autotune;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.List;
import java.util.Locale;

/**
 * Finds the velocity PIDF coefficients for a flywheel or shooter without any manual input.
 * Call {@link #update()} every loop until {@link #isDone()}, then read {@link #getResult()}.
 *
 * <pre>
 * CONFIRM_DIRECTION  spin each motor alone; the user confirms or reverses it
 * PROBE              run at full power to measure the real top speed (REV's own method). F should be close
 *                    to 32767 / top speed, which also sets the scale for the searches below
 * TUNE_F             find the largest F that tracks just under the target speeds
 * TUNE_GAINS         P: raise it until a step overshoots slightly. I and D are searched only if validation
 *                    shows they are needed, one after the other
 * VALIDATE           follow the full profile (holds, ramps, random targets) and check the error
 *
 * Each of F, P, I and D is found with the same search. It starts small and multiplies by 10 until the value
 * goes over, which brackets it between the last value that did not go over and the first that did. The
 * bracket is then narrowed by halving, on a log scale so that every trial adds about another digit, until it
 * is tight, and the largest value that did not go over is kept. If even the first value goes over, it is
 * divided by 10 until it does not. The climb never skips the upper limit, it tests it. Gains are never
 * negative, and speeds are read as absolute values so the direction of rotation does not matter.
 *
 * Every leg of the path ends when the flywheel settles, not after a fixed time, so a flywheel that
 * takes several seconds to converge is handled the same as a fast one.
 *
 * Tuning stops by itself after {@link Config#maxTotalSeconds}, and {@link #getWarning()} reports
 * problems as they appear (a target that will not settle, not enough power to reach a target, a low
 * battery) so the user can stop early.
 */
public class FlywheelAutoTuner {
    public static class Config {
        // Direction check: spin each motor alone and have the user confirm the way it turns
        public double confirmPower = 0.2;
        public double confirmTimeoutSeconds = 60;

        // Full-power run that measures the top speed
        public double probePower = 1.0;
        public double probeMinSeconds = 2.0;
        public double probeMaxSeconds = 10.0;
        public double coastMaxSeconds = 15.0;      // waiting for the flywheel to slow down afterwards
        public double lowBatteryVolts = 11.5;

        // Time limit for everything after the direction check
        public double maxTotalSeconds = 300;      // the whole tune should take less than 5 minutes
        public double reserveSeconds = 60;        // when this little time is left, stop searching and validate with what was found

        // The path (fractions of max velocity). Ramps and the random targets are seeded.
        // All fractions are of the measured top speed. The band runs from two thirds up to 95%: 100% is full
        // power, which leaves the feedback nothing to add, so it cannot be tracked. Two thirds is what 1600 is of a
        // 2400 ticks/s top speed, the speed the owner wanted, so it scales with whatever top speed is measured.
        public double profileLowFraction = 0.2;     // idle: where each spin-up starts from. Nothing below the band is scored
        public double profileCentreFraction = 2.0 / 3; // the speed each spin-up goes to unless an operating speed is given
        public double profileHighFraction = 0.95;   // top of the band
        public double randomLowFraction = 2.0 / 3;  // bottom of the band
        public double riseFraction = 0.75;          // the spin-up is timed to this fraction of the top speed
        public double minStepGapFraction = 0.25;  // random targets differ by at least this much
        public long profileSeed = 12345;
        public double rampUpSeconds = 4;
        public double rampDownSeconds = 3;
        public int validateRandomTargets = 2;

        // Deciding a leg is finished
        // A leg is finished when the newer half of the window matches the older half. With working feedback a
        // spin-up settles quickly. The F search has P, I and D at 0, so it is open loop and needs longer.
        public double steadyWindowSeconds = 1.5;
        public double steadyBandFraction = 0.002; // of top speed, and never tighter than the reading noise allows
        public double steadyBandMaxFraction = 0.006; // ... but noise can never widen it past this, so jitter cannot end a leg early
        public double minLegSeconds = 1.5;
        public double legTimeoutSeconds = 8;
        public double fMinLegSeconds = 2.5;        // the F search, which has no feedback
        public double fLegTimeoutSeconds = 15;

        // The search used for F, P, I and D (see the class comment)
        public double[] fSetPoints = {0.95, 2.0 / 3}; // fractions of top speed, the top and bottom of the band (set again from the band once the top speed is known)
        public double fStartFraction = 1e-3;          // F starts at this fraction of 32767 / top speed
        public double pStartFractionOfF = 1e-4;
        public double iStartFractionOfP = 1e-3;
        public double dStartFractionOfP = 1e-3;
        public double fMaxFraction = 100;             // F goes up to this many times 32767 / top speed
        public double pMaxFractionOfF = 20;           // P stops climbing here if nothing has overshot (268 at F=13.4)
        public double iMaxFractionOfP = 1;
        public double dMaxFractionOfP = 0.5;
        public double searchFloorFraction = 1e-6;     // going lower than start * this gives up
        public double fPrecision = 0.005;             // F: stop when the bracket is this tight (0.5%)
        public double gainPrecision = 0.02;           // P, I, D: the same, 2%
        public int maxSearchTrials = 60;              // per value
        public double fAimRatio = 0.98;               // while climbing, aim each step at this share of the target speed
        public double gainStepFine = 2.5;             // P, I, D step once the gain is starting to change the spin-up time
        public double effectFraction = 0.03;          // a change in spin-up time bigger than this counts as an effect
        // Optional second limit on P, off by default. The error that commands full power is 32767 / P, and once that is only a
        // few reading steps wide the controller is switching on and off on reading noise. 0.17 would keep the proportional band
        // at least six steps wide (about P = 278 at a 20 ticks/s step), which is close to where 20 x F already ends the climb.
        public double maxPowerPerReadingStep = 0;     // 0 = off
        public double settleBand = 0.02;              // the spin-up is timed to getting within this share of the target
        public double rampTolerance = 0.012;          // mean ramp-following error, as a share of top speed, that counts as following the curve
        public double iBiasTrigger = 0.01;            // I is tuned when the average error is more than this off the target, even if it is within the tolerance
        public double plateauFraction = 0;            // 0 = off. If set, P stops once a bigger P no longer settles the speed this much sooner
        public double idleReturnTimeoutSeconds = 4;   // the unmeasured step back to idle only has to get close
        public double fOverTolerance = 0.003;         // a speed this far over the target counts as over
        public double maxOvershoot = 0.02;            // fraction of the TARGET speed. A shooter has to be consistent
        public double overshootMargin = 0.01;         // I or D may add this much overshoot on top of what P alone leaves
        public int maxFlips = 3;                      // error sign changes that count as ringing
        public double bandFraction = 0.02;            // of top speed; error inside this is ignored
        public double pBackoff = 0.9;                 // final value = this * the largest that did not go over
        public double iBackoff = 0.9;
        public double dBackoff = 0.9;
        public double feedbackConfirmSeconds = 1.5;   // how long an overshoot is watched for coming back down

        // Validation
        public double validateTolerance = 0.02;   // mean abs error of the holds, as a fraction of target
        public double biasCorrectionThreshold = 0.02;
    }

    public enum Phase { CONFIRM_DIRECTION, PROBE, TUNE_F, TUNE_GAINS, VALIDATE, DONE, FAILED }

    private final TunableMotor motor;
    private final Config cfg;
    private double max;                  // top speed, measured at full power
    private StepMetrics metrics;
    private final SteadyWindow window;
    private final ElapsedTime clock = new ElapsedTime();
    private final StringBuilder notes = new StringBuilder();
    private java.io.BufferedWriter log;
    private double lastLogFlush;

    private Phase phase;
    private boolean entered;
    private double start;           // start of the current leg (or probe)
    private double tuneStart = Double.NaN; // when the time limit started
    private double probeSum;
    private int probeCount;
    private boolean coasting;
    private double fRef;                 // 32767 / top speed, the formula REV gives for F
    private double operatingSpeed;       // ticks per second the mechanism is actually used at, 0 for the full range
    private boolean velocityFlipped;   // the readout is negative for positive power
    private boolean encoderInverted;   // the hub's feedback pushes the wrong way, so P is skipped
    private String legWarning, diagnosis;
    private int legTimeouts;
    private int legsSettled;
    private double legSecondsTotal, noiseSum;
    private int noiseCount;
    private final StringBuilder searchReport = new StringBuilder();
    private boolean legTimedOut;
    private double target;
    private String status = "Starting";
    private FlywheelTuneResult result;

    private double p, i, d, f;

    // Direction check
    private int dirIndex;
    private boolean dirSpinning;
    private double dirSpeed;               // smoothed raw speed of the motor being checked
    private double[] encoderSign;          // +1 if a motor's own encoder counts up for positive power, -1 if down, 0 unknown

    private boolean slowNoted;

    // The search
    private enum Param { F, P, I, D }
    private Param param;
    private double value;            // the value being tried
    private double lo, hi = Double.NaN; // largest value not over (0 until found), smallest value over
    private int trials;
    private boolean preSettle;
    private boolean recovering;        // settling after a trial that was cut short
    private int trialLeg;              // 0 = the step up, 1 = the step back down
    private boolean baseline;          // measuring what P alone leaves, before searching I or D
    private double baseOvershoot, trialMaxOvershoot;
    private double trialRise = -1;     // seconds the step up took to reach the rise fraction of its target
    private double riseTime = -1;      // the same, for the spin-up in validation
    private double trialSettle = -1;   // seconds the step up took to get within the settle band of its target
    private double settleTime = -1;    // the same, for the spin-up in validation
    private int baseFlips, trialMaxFlips;
    private double pendingValue;     // the value to try once it has settled
    private double overshootSince;
    private int fSetIndex;
    private double lastRawReading = Double.NaN;
    // Battery voltage and motor current are separate hub reads, so they are sampled at about 10 Hz instead of every loop
    private double lastSlowSample = -1, cachedVolts = Double.NaN, minVolts = Double.MAX_VALUE, maxVolts = -Double.MAX_VALUE;
    private double[] cachedAmps;
    private double readingStep = Double.MAX_VALUE;   // the smallest change seen between two different speed readings
    private double fTrialRatio, loRatio, hiRatio;  // speed / target, the worst across the set points of a trial
    private double prevRise = -1;
    private double firstRise = -1, bestRise = -1, bestRiseValue, plateauValue = -1;   // P stops when the spin-up stops getting faster
    private double legTimeoutOverride = -1;

    // Validate
    private List<TargetProfile.Leg> legs;
    private int legIndex;
    private double holdSumAbs, holdSumSigned, rampSumAbs;
    private int holdCount, rampCount;
    private double lastMeanError = Double.NaN, lastRampError = Double.NaN;
    private boolean fCorrected, iTuned, dTuned;
    private double valMaxOvershoot, valOtherOvershoot;
    private boolean valRinging;
    private StringBuilder holdReport = new StringBuilder();

    public FlywheelAutoTuner(TunableMotor motor) {
        this.motor = motor;
        this.cfg = new Config();
        this.max = motor.getMaxVelocity(); // the motor's rated speed, replaced by the measured one
        this.metrics = new StepMetrics(cfg.bandFraction * max);
        this.window = new SteadyWindow(cfg.steadyWindowSeconds);
        this.phase = Phase.CONFIRM_DIRECTION;
        this.encoderSign = new double[motor.getMotorCount()];
        this.cachedAmps = new double[motor.getMotorCount()];
    }

    /**
     * The speed the mechanism is actually used at, in ticks per second. F is fitted just under it, and the steps
     * and the validation profile are built around it instead of spanning the whole speed range. Call before the
     * first update. Leave unset to tune across the full range.
     */
    public void setOperatingSpeed(double ticksPerSecond) {
        this.operatingSpeed = Math.max(0, ticksPerSecond);
    }

    public FlywheelAutoTuner(HardwareMap hw, String... motorNames) {
        this(new MotorGroup(hw, motorNames));
    }

    /**
     * Writes every sample (time, phase, the value being searched, target, speed, gains) to a CSV file, so a
     * run can be looked at afterwards. Call before the first update. Returns false if the file could not be created.
     */
    public boolean setLogFile(java.io.File file) {
        try {
            if (file.getParentFile() != null) file.getParentFile().mkdirs();
            log = new java.io.BufferedWriter(new java.io.FileWriter(file));
            StringBuilder header = new StringBuilder("time,phase,param,value,target,velocity,rawVelocity,p,i,d,f,volts");
            // By role, because the motor that closes the loop can change after the direction check
            for (int k = 0; k < motor.getMotorCount(); k++) header.append(k == 0 ? ",amps_primary" : ",amps_follower" + k);
            log.write(header.append(",note\n").toString());
            log.flush();
            return true;
        } catch (java.io.IOException | SecurityException e) {
            log = null;
            return false;
        }
    }

    private void sampleSlow(double now) {
        if (now - lastSlowSample < 0.1) return;
        lastSlowSample = now;
        double volts = motor.getVoltage();
        if (!Double.isNaN(volts)) {
            cachedVolts = volts;
            minVolts = Math.min(minVolts, volts);
            maxVolts = Math.max(maxVolts, volts);
        }
        for (int k = 0; k < cachedAmps.length; k++) cachedAmps[k] = motor.getCurrentAmps(k);
    }

    private void logRow(double now, String note) {
        if (log == null) return;
        try {
            StringBuilder amps = new StringBuilder();
            for (double a : cachedAmps) amps.append(String.format(Locale.US, ",%.2f", a));
            log.write(String.format(Locale.US, "%.3f,%s,%s,%.6g,%.1f,%.1f,%.1f,%.6g,%.6g,%.6g,%.6g,%.2f%s,%s\n", now, phase,
                    param == null ? "" : param.name(), value, target, velocity(), motor.getVelocity(), p, i, d, f,
                    cachedVolts, amps, note));
            if (now - lastLogFlush > 2) {
                log.flush();
                lastLogFlush = now;
            }
        } catch (java.io.IOException e) {
            log = null;
        }
    }

    private void closeLog() {
        if (log == null) return;
        try {
            log.close();
        } catch (java.io.IOException ignored) {
        }
        log = null;
    }

    public void update() {
        double now = clock.seconds();
        if (phase != Phase.CONFIRM_DIRECTION && !isDone()) {
            double raw = motor.getVelocity();
            double change = Math.abs(raw - lastRawReading);
            if (change > 0 && change < readingStep) readingStep = change;
            lastRawReading = raw;
            sampleSlow(now);
            logRow(now, "");
        }
        if (!Double.isNaN(tuneStart) && !isDone() && now - tuneStart > cfg.maxTotalSeconds) {
            fail(String.format(Locale.US, "Tuning took more than %.0f minutes. Check for friction, a slipping belt or a weak battery.",
                    cfg.maxTotalSeconds / 60));
            return;
        }
        switch (phase) {
            case CONFIRM_DIRECTION: confirmDirection(now); break;
            case PROBE:    probe(now); break;
            case TUNE_F:   tuneF(now); break;
            case TUNE_GAINS: tuneGain(now); break;
            case VALIDATE: validate(now); break;
            default: break;
        }
    }

    public boolean isDone() { return phase == Phase.DONE || phase == Phase.FAILED; }
    public Phase getPhase() { return phase; }
    public String getStatus() { return status; }
    public double getTarget() { return target; }
    public double getVelocity() { return velocity(); }
    /** The coefficients currently applied to the motor(s), for live display. */
    public double[] getCurrentPIDF() { return new double[]{p, i, d, f}; }
    /** Seconds left before tuning gives up. The clock starts after the direction check. */
    public double getSecondsRemaining() {
        return Double.isNaN(tuneStart) ? cfg.maxTotalSeconds
                : Math.max(0, cfg.maxTotalSeconds - (clock.seconds() - tuneStart));
    }

    /** A problem the user may want to stop for, or null if things look fine. */
    public String getWarning() {
        if (diagnosis != null) return diagnosis;
        if (legWarning != null) return legWarning;
        if (legTimeouts > 0) return legTimeouts + " target(s) did not settle so far";
        return null;
    }

    /** Null until {@link #isDone()}. */
    public FlywheelTuneResult getResult() { return result; }

    public void abort() {
        if (!isDone()) fail("Aborted");
    }

    /** Measured speed, flipped when the readout is negative for positive power. */
    private double velocity() {
        double v = motor.getVelocity();
        return velocityFlipped ? -v : v;
    }

    // ---- legs ----

    private void beginLeg(double now, double newTarget) {
        target = newTarget;
        start = now;
        window.reset();
        legTimedOut = false;
        legWarning = null;
        legTimeoutOverride = -1;
        motor.setVelocity(newTarget);
        logRow(now, "leg start");
    }

    private void noteLegEnd(double seconds) {
        legSecondsTotal += seconds;
        if (window.isFull()) {
            noiseSum += window.noise();
            noiseCount++;
        }
    }

    private String diagnostics() {
        int legs = legsSettled + legTimeouts;
        if (legs == 0) return "";
        double noise = noiseCount > 0 ? noiseSum / noiseCount : 0;
        String battery = minVolts == Double.MAX_VALUE ? ""
                : String.format(Locale.US, "Battery %.1f V at its highest and %.1f V at its lowest. ", maxVolts, minVolts);
        return battery + String.format(Locale.US, "%d legs settled, %d timed out, %.1f s each on average. Speed reading jitter about %.0f ticks/s "
                + "(%.1f%% of top speed); a leg counts as settled when it drifts less than %.0f ticks/s.",
                legsSettled, legTimeouts, legSecondsTotal / legs, noise, 100 * noise / max,
                Math.min(cfg.steadyBandMaxFraction * max, Math.max(cfg.steadyBandFraction * max, 3 * noise * 2 / Math.sqrt(2.0 * 50))));
    }

    private double steadyBand() {
        return Math.min(Math.max(cfg.steadyBandFraction * max, window.noiseBand()), cfg.steadyBandMaxFraction * max);
    }

    private boolean windowSteady() {
        return window.isFull()
                && Math.abs(window.drift()) <= steadyBand();
    }

    /** Samples the velocity, and returns true once the leg has settled or timed out. */
    /** 75% of the top speed, but never above 90% of the speed being aimed for, which it could never be timed to. */
    /** How long the last spin-up took to get within the settle band, or the leg timeout if it never did. */
    private double trialMetric() {
        return trialSettle >= 0 ? trialSettle : cfg.legTimeoutSeconds;
    }

    private double riseThreshold() {
        return Math.min(cfg.riseFraction * max, 0.9 * target);
    }

    private double legTimeout() {
        if (legTimeoutOverride > 0) return legTimeoutOverride;
        return phase == Phase.TUNE_F ? cfg.fLegTimeoutSeconds : cfg.legTimeoutSeconds;
    }

    private boolean budgetLow(double now) {
        return !Double.isNaN(tuneStart) && cfg.maxTotalSeconds - (now - tuneStart) < cfg.reserveSeconds;
    }

    private boolean legDone(double now) {
        window.add(now, velocity());
        double t = now - start;
        double timeout = legTimeout();
        if (t >= timeout) {
            legTimedOut = true;
            legTimeouts++;
            noteLegEnd(t);
            return true;
        }
        if (t >= 0.6 * timeout) {
            legWarning = String.format(Locale.US,
                    "Slow to settle: waited %.0fs for this target. Friction, a weak battery or noise?", t);
        }
        if (t >= (phase == Phase.TUNE_F ? cfg.fMinLegSeconds : cfg.minLegSeconds) && windowSteady()) {
            legsSettled++;
            noteLegEnd(t);
            return true;
        }
        return false;
    }

    // ---- CONFIRM_DIRECTION ----

    /** The motor now spinning turns the right way. Moves on to the next motor. */
    public void confirmDirection() {
        if (phase != Phase.CONFIRM_DIRECTION) return;
        motor.stop();
        // setDirection flips the power and the encoder together, so this does not depend on which way the user chose
        encoderSign[dirIndex] = Math.abs(dirSpeed) < 15 ? 0 : Math.signum(dirSpeed);
        dirIndex++;
        dirSpinning = false;
        if (dirIndex >= motor.getMotorCount()) {
            chooseClosingMotor();
            phase = Phase.PROBE;
            entered = false;
        }
    }

    /** The motor now spinning turns the wrong way. Flips it and spins it again. */
    public void reverseDirection() {
        if (phase != Phase.CONFIRM_DIRECTION) return;
        motor.reverseMotor(dirIndex);
        dirSpinning = false;
    }

    /**
     * Only one motor closes the speed loop, and that has to be one whose own encoder counts the right way. If one
     * motor's encoder reads backwards and another's does not, the one that does is made first.
     */
    private void chooseClosingMotor() {
        int closing = 0;
        for (int k = 0; k < encoderSign.length; k++) {
            if (encoderSign[k] > 0) {
                closing = k;
                break;
            }
        }
        StringBuilder report = new StringBuilder();
        for (int k = 0; k < encoderSign.length; k++) {
            if (report.length() > 0) report.append(", ");
            report.append(motor.getMotorName(k)).append(encoderSign[k] > 0 ? " counts the right way"
                    : encoderSign[k] < 0 ? " counts BACKWARDS" : " could not be read");
        }
        if (closing != 0) {
            String name = motor.getMotorName(closing);
            motor.setPrimary(closing);
            double swap = encoderSign[0];
            encoderSign[0] = encoderSign[closing];
            encoderSign[closing] = swap;
            notes.append("Encoders: ").append(report).append("; so ").append(name).append(" closes the speed loop; ");
        } else {
            notes.append("Encoders: ").append(report).append("; ");
        }
    }

    private void confirmDirection(double now) {
        if (!dirSpinning) {
            dirSpinning = true;
            start = now;
            dirSpeed = 0;
            motor.spinMotor(dirIndex, cfg.confirmPower);
        }
        dirSpeed += 0.15 * (motor.getMotorVelocity(dirIndex) - dirSpeed);
        status = String.format(Locale.US, "Motor '%s' (%d/%d) is spinning. Confirm if it turns the right way, otherwise reverse it.",
                motor.getMotorName(dirIndex), dirIndex + 1, motor.getMotorCount());
        if (now - start > cfg.confirmTimeoutSeconds) fail("No answer to the direction check.");
    }

    // ---- PROBE ----

    private void probe(double now) {
        if (coasting) {
            // Let the flywheel slow down so the first F trial does not start with it still spinning
            status = "Letting the flywheel coast down";
            if (Math.abs(motor.getVelocity()) < 0.05 * max || now - start > cfg.coastMaxSeconds) startF(now);
            return;
        }

        if (!entered) {
            entered = true;
            start = now;
            tuneStart = now;
            probeSum = 0;
            probeCount = 0;
            window.reset();
            motor.setOpenLoopPower(cfg.probePower);
        }
        status = "Measuring top speed (the flywheel runs at full power)";
        double t = now - start;
        double raw = motor.getVelocity();
        window.add(now, Math.abs(raw));
        if (t >= 1.0) {
            probeSum += raw;
            probeCount++;
        }
        boolean steady = t >= cfg.probeMinSeconds && windowSteady();
        if (!steady && t < cfg.probeMaxSeconds) return;

        double topSpeed = window.mean();
        double signedAverage = probeCount > 0 ? probeSum / probeCount : 0;
        double volts = motor.getVoltage();
        motor.stop();
        if (volts < cfg.lowBatteryVolts) {
            diagnosis = String.format(Locale.US, "Battery is %.1f V. Use a fully charged battery.", volts);
        }
        if (topSpeed < 0.1 * cfg.probePower * max) {
            fail(String.format(Locale.US, "Encoder barely moved (%.0f ticks/s at full power). Check the encoder cable.", topSpeed));
            return;
        }
        if (signedAverage < 0) {
            // setDirection flips the encoder along with the power, so changing it cannot fix this.
            // Read the speed as positive, and tune on its size.
            velocityFlipped = true;
            notes.append(String.format(Locale.US, "Velocity reads negative for positive power, read as positive; "));
        }
        max = topSpeed;
        metrics = new StepMetrics(cfg.bandFraction * max);
        fRef = 32767.0 / max;
        notes.append(String.format(Locale.US, "Top speed %.0f ticks/s, so REV's formula gives F=%.3f; ", max, fRef));
        // F must stay under the target at the top and the bottom of the band
        cfg.fSetPoints = new double[]{cfg.profileHighFraction, cfg.randomLowFraction};
        if (operatingSpeed > 0) centreOnOperatingSpeed();
        coasting = true;
        start = now;
    }

    /** Builds the set points, the steps and the validation profile around the speed the mechanism is used at. */
    private void centreOnOperatingSpeed() {
        if (operatingSpeed > cfg.profileHighFraction * max) {
            String msg = String.format(Locale.US, "The operating speed %.0f is %.0f%% of the top speed, above the %.0f%% the tuner can track.",
                    operatingSpeed, 100 * operatingSpeed / max, 100 * cfg.profileHighFraction);
            diagnosis = diagnosis == null ? msg : diagnosis + " " + msg;
        }
        // The band that is scored runs from the operating speed up to 95% of the top speed. It does not start
        // at 75% of the top speed if the operating speed is lower than that.
        double at = Math.min(operatingSpeed / max, cfg.profileHighFraction);
        cfg.randomLowFraction = Math.min(cfg.randomLowFraction, at);
        cfg.profileCentreFraction = at;
        cfg.profileLowFraction = Math.min(cfg.profileLowFraction, 0.5 * at);
        cfg.fSetPoints = new double[]{cfg.profileHighFraction, cfg.randomLowFraction};
        notes.append(String.format(Locale.US, "Spin-ups go to %.0f ticks/s, and the scored band is %.0f to %.0f ticks/s; ",
                cfg.profileCentreFraction * max, cfg.randomLowFraction * max, cfg.profileHighFraction * max));
    }

    // ---- the search ----

    private void startF(double now) {
        phase = Phase.TUNE_F;
        p = i = d = 0;
        startSearch(now, Param.F);
    }

    private void startGains(double now) {
        phase = Phase.TUNE_GAINS;
        startSearch(now, Param.P);
    }

    private void startSearch(double now, Param which) {
        param = which;
        lo = 0;
        hi = Double.NaN;
        trials = 0;
        fTrialRatio = 0;
        loRatio = 0;
        hiRatio = 0;
        prevRise = -1;
        firstRise = -1;
        bestRise = -1;
        plateauValue = -1;
        if (searchReport.length() > 0) searchReport.append("\n");
        searchReport.append(param).append(": ");
        value = startValue();
        setParam(value);
        motor.setPIDF(p, i, d, f);
        if (param == Param.F) {
            fSetIndex = 0;
            beginLeg(now, cfg.fSetPoints[0] * max);
        } else {
            phase = Phase.TUNE_GAINS;
            // I and D are judged against what the gains found so far already leave behind, so measure that
            // first with this gain at 0. P is judged against a flywheel with no feedback, which has none.
            baseline = param != Param.P;
            baseOvershoot = 0;
            baseFlips = 0;
            if (baseline) setParam(0);
            motor.setPIDF(p, i, d, f);
            preSettle = true;
            beginLeg(now, cfg.profileLowFraction * max);
        }
    }

    /** F: hold each set point and see whether this F takes the flywheel past any of them. */
    private void tuneF(double now) {
        status = String.format(Locale.US, "F: trial %d, F=%s, set point %d/%d (largest not over: %s)",
                trials + 1, fmt(value), fSetIndex + 1, cfg.fSetPoints.length, fmt(lo));
        if (!legDone(now)) return;
        noteIfSlow();
        fTrialRatio = Math.max(fTrialRatio, window.mean() / target);
        boolean over = window.mean() > target * (1 + cfg.fOverTolerance);
        if (over || ++fSetIndex >= cfg.fSetPoints.length) {
            fSetIndex = 0;
            advance(now, over);
        } else {
            beginLeg(now, cfg.fSetPoints[fSetIndex] * max);
        }
    }

    /** P, I or D: step between random targets and see whether this value overshoots or rings. */
    /**
     * P, I or D. Every trial is the same spin-up step from idle, and the only thing that changes from one trial
     * to the next is the gain. A step back down to idle then resets the flywheel for the next trial. That step
     * is not measured: for a shooter it is the spin-up that matters.
     */
    private void tuneGain(double now) {
        if (preSettle) {
            status = recovering
                    ? String.format(Locale.US, "%s: letting the speed settle before trial %d", param, trials + 1)
                    : "Settling at the idle speed before tuning " + param;
            if (legDone(now)) {
                preSettle = false;
                if (recovering) {
                    recovering = false;
                    setParam(pendingValue);
                }
                startTrial(now);
            }
            return;
        }

        if (trialLeg == 1) {
            status = String.format(Locale.US, "%s: back to idle for the next trial", param);
            if (!legDone(now)) return;
            if (baseline) {
                baseline = false;
                baseOvershoot = trialMaxOvershoot;
                baseFlips = trialMaxFlips;
                prevRise = trialRise;
                value = startValue();
                setParam(value);
                startTrial(now);
            } else {
                advance(now, false);
            }
            return;
        }

        metrics.add(velocity());
        if (trialRise < 0 && metrics.smoothed() >= riseThreshold()) trialRise = now - start;
        if (trialSettle < 0 && Math.abs(metrics.smoothed() / target - 1) <= cfg.settleBand) trialSettle = now - start;
        trialMaxOvershoot = Math.max(trialMaxOvershoot, metrics.overshootOfTarget());
        trialMaxFlips = Math.max(trialMaxFlips, metrics.flips());
        String rise = trialRise >= 0 ? String.format(Locale.US, ", %.0f%% in %.2fs", 100 * cfg.riseFraction, trialRise) : "";
        status = baseline
                ? String.format(Locale.US, "%s: baseline with %s at 0%s", param, param, rise)
                : String.format(Locale.US, "%s: trial %d, %s=%s%s (largest not over: %s)",
                        param, trials + 1, param, fmt(value), rise, fmt(lo));

        double overshootLimit = Math.max(cfg.maxOvershoot, baseOvershoot + cfg.overshootMargin);
        int flipLimit = Math.max(cfg.maxFlips, baseFlips + 2);
        boolean ringing = metrics.flips() >= flipLimit;
        boolean over = metrics.overshootOfTarget() > overshootLimit;
        if (!baseline) {
            if (over && !ringing && overshootSince < 0) overshootSince = now;
            if (over && !ringing && param == Param.P && velocityFlipped && metrics.isStepUp()) {
                // Feedback that works backwards holds the speed above the target. A healthy overshoot comes back down.
                if (now - overshootSince < cfg.feedbackConfirmSeconds) return;
                if (metrics.error() > 0.5 * metrics.peakOvershoot()) {
                    feedbackBackwards(now);
                    return;
                }
            }
        }

        // A trial that is already ringing or overshooting does not need to be waited out.
        // D is judged on ringing only: it exists to calm an overshoot that P and I leave behind.
        boolean unstable = !baseline && (ringing || (over && param != Param.D));
        if (unstable) {
            advance(now, true);
            return;
        }
        if (!legDone(now)) return;

        // The spin-up has settled. Go back to idle for the next trial.
        trialLeg = 1;
        beginLeg(now, cfg.profileLowFraction * max);
        legTimeoutOverride = cfg.idleReturnTimeoutSeconds;
    }

    /** Records whether the value just tried went over, then moves to the next value or finishes. */
    private void advance(double now, boolean over) {
        logRow(now, String.format(Locale.US, "trial result %s=%.6g over=%b", param, value, over));
        searchReport.append(String.format(Locale.US, "%.3g %s, ", value, over ? "over" : "under"));
        trials++;
        if (over) {
            if (Double.isNaN(hi) || value < hi) {
                hi = value;
                hiRatio = fTrialRatio;
            }
        } else if (value > lo) {
            lo = value;
            loRatio = fTrialRatio;
        }

        if (param == Param.P && !over && cfg.plateauFraction > 0) {
            // What matters is how soon the speed is within the band of the target, not how soon it is near it
            double metric = trialMetric();
            if (firstRise < 0) firstRise = metric;
            if (bestRise < 0 || metric < bestRise * (1 - cfg.plateauFraction)) {
                bestRise = metric;
                bestRiseValue = value;
            } else if (bestRise < firstRise * (1 - cfg.effectFraction)) {
                // The gain was making the speed settle sooner and now it is not, so there is nothing more to gain
                plateauValue = bestRiseValue;
            }
        }
        double next = nextValue(now);
        prevRise = trialRise;
        fTrialRatio = 0;
        if (Double.isNaN(next)) {
            finishSearch(now);
            return;
        }
        value = next;
        if (param == Param.F) {
            setParam(value);
            motor.setPIDF(p, i, d, f);
            beginLeg(now, cfg.fSetPoints[0] * max);
        } else if (over) {
            // That trial was cut short while the speed was still moving. Go back to the low target under the
            // last value known to be safe and wait for it to settle, so the next step up starts from rest and
            // is not thrown off by what is left of this one.
            pendingValue = value;
            setParam(lo);
            motor.setPIDF(p, i, d, f);
            recovering = true;
            preSettle = true;
            beginLeg(now, cfg.profileLowFraction * max);
            legTimeoutOverride = cfg.idleReturnTimeoutSeconds;
        } else {
            setParam(value);
            startTrial(now);
        }
    }

    /**
     * The next value to try, or NaN when the search is done. Far from the answer it climbs by 10. Near it, it
     * steps by what the measurements predict instead, so it cannot leap past the answer by a factor of 10.
     */
    private double nextValue(double now) {
        if (trials >= cfg.maxSearchTrials) {
            notes.append(param).append(": hit the trial limit; ");
            return Double.NaN;
        }
        if (plateauValue > 0) return Double.NaN;
        if (lo > 0 && budgetLow(now)) {
            notes.append(param).append(": ran short of time, used the best value so far; ");
            return Double.NaN;
        }
        if (lo == 0) {
            // Even the first value went over, so go lower for more precision
            double smaller = value / 10;
            return smaller < startValue() * cfg.searchFloorFraction ? Double.NaN : smaller;
        }
        if (Double.isNaN(hi)) {
            // Nothing has gone over yet. The upper limit is tested too, never jumped over, so the search
            // cannot give up short of it.
            if (value >= maxValue()) return Double.NaN;
            double factor;
            if (param == Param.F) {
                // The speed is close to proportional to F, so the ratio says how far to go
                factor = fTrialRatio > 0.05 ? clamp(cfg.fAimRatio / fTrialRatio, 1.02, 10) : 10;
            } else {
                // A gain that is not changing the spin-up time yet can be climbed quickly. Once it is, go carefully.
                boolean effect = prevRise > 0 && trialRise > 0
                        && Math.abs(trialRise - prevRise) / prevRise >= cfg.effectFraction;
                factor = effect ? cfg.gainStepFine : 10;
            }
            return Math.min(value * factor, maxValue());
        }
        if (hi / lo <= 1 + (param == Param.F ? cfg.fPrecision : cfg.gainPrecision)) return Double.NaN;
        if (param == Param.F && hiRatio > 0) {
            // Aim at where the speed meets the target, but never so close to an end that the bracket barely shrinks.
            // If the low end had barely started the flywheel, its ratio says nothing, so scale back from the over point.
            double width = hi - lo;
            double predicted = loRatio >= 0.05 && hiRatio > loRatio
                    ? lo + (1 - loRatio) * width / (hiRatio - loRatio)
                    : hi * cfg.fAimRatio / hiRatio;
            return clamp(predicted, lo + 0.25 * width, hi - 0.25 * width);
        }
        return Math.sqrt(lo * hi);
    }

    private void startTrial(double now) {
        trialLeg = 0;
        trialMaxOvershoot = 0;
        trialMaxFlips = 0;
        trialRise = -1;
        trialSettle = -1;
        double up = cfg.profileCentreFraction * max;
        motor.setPIDF(p, i, d, f);
        metrics.begin(up, velocity());
        overshootSince = -1;
        beginLeg(now, up);
    }

    private void finishSearch(double now) {
        if (param == Param.F) {
            if (lo == 0) {
                fail("F went over the target even at its smallest value. Check the encoder and the motor.");
                return;
            }
            if (Double.isNaN(hi)) {
                fail(String.format(Locale.US, "Never reached the %.0f%% set point even with F far past full power. "
                        + "Check friction, the load, the belt and the battery.", 100 * cfg.fSetPoints[0]));
                return;
            }
            f = lo;
            notes.append(String.format(Locale.US, "F=%.3f (REV formula %.3f, %+.0f%%); ", f, fRef, 100 * (f / fRef - 1)));
            startGains(now);
            return;
        }

        if (param == Param.P && plateauValue > 0) {
            setParam(plateauValue);
            notes.append(String.format(Locale.US, "P stopped rising once the speed stopped settling sooner (%.2f s to get within %.0f%% of the target); ",
                    bestRise, 100 * cfg.settleBand));
            startValidate(now);
            return;
        }
        if (lo == 0) {
            if (param == Param.P) {
                fail("Could not find a stable P. Check for slipping or a loose flywheel.");
                return;
            }
            setParam(0);
            notes.append(param).append(" left at 0 (no value found that did not go over); ");
        } else if (Double.isNaN(hi) && param != Param.P) {
            // No limit was found up to the cap, so there is nothing to say what this value should be
            setParam(0);
            notes.append(param).append(" never reached a limit, so it was left at 0; ");
        } else {
            setParam(Double.isNaN(hi) ? lo : lo * backoff());
        }
        startValidate(now);
    }

    private void feedbackBackwards(double now) {
        encoderInverted = true;
        p = i = d = 0;
        motor.setPIDF(0, 0, 0, f);
        String msg = "Adding P pushed the speed up and held it above the target, so the motor's velocity "
                + "feedback works backwards. P, I and D cannot help, so only F is tuned.";
        diagnosis = diagnosis == null ? msg : diagnosis + " " + msg;
        notes.append("Velocity feedback works backwards, so only F was tuned; ");
        startValidate(now);
    }

    /** Gains are never negative. */
    private void setParam(double v) {
        v = Math.max(0, v);
        switch (param) {
            case F: f = v; break;
            case P: p = v; break;
            case I: i = v; break;
            default: d = v;
        }
    }

    private double startValue() {
        switch (param) {
            case F: return fRef * cfg.fStartFraction;
            case P: return f * cfg.pStartFractionOfF;
            case I: return p * cfg.iStartFractionOfP;
            default: return p * cfg.dStartFractionOfP;
        }
    }

    private double maxValue() {
        switch (param) {
            case F: return fRef * cfg.fMaxFraction;
            case P: {
                // One step in the speed reading must not swing the power by more than a few percent, or the motor chatters
                if (cfg.maxPowerPerReadingStep <= 0) return f * cfg.pMaxFractionOfF;
                double step = readingStep == Double.MAX_VALUE ? 20 : readingStep;
                return Math.min(f * cfg.pMaxFractionOfF, cfg.maxPowerPerReadingStep * 32767.0 / step);
            }
            case I: return p * cfg.iMaxFractionOfP;
            default: return p * cfg.dMaxFractionOfP;
        }
    }

    private double backoff() {
        switch (param) {
            case P: return cfg.pBackoff;
            case I: return cfg.iBackoff;
            default: return cfg.dBackoff;
        }
    }

    private static String fmt(double v) {
        return v == 0 ? "none" : String.format(Locale.US, "%.3g", v);
    }

    private void noteIfSlow() {
        if (legTimedOut && !slowNoted) {
            slowNoted = true;
            notes.append(String.format(Locale.US, "A target did not settle within %.0fs; ", legTimeout()));
        }
    }

    // ---- VALIDATE ----

    private void startValidate(double now) {
        phase = Phase.VALIDATE;
        legs = TargetProfile.validation(cfg);
        legIndex = 0;
        holdSumAbs = 0;
        holdSumSigned = 0;
        holdCount = 0;
        rampSumAbs = 0;
        rampCount = 0;
        holdReport = new StringBuilder();
        valMaxOvershoot = 0;
        valOtherOvershoot = 0;
        valRinging = false;
        riseTime = -1;
        settleTime = -1;
        motor.setPIDF(p, i, d, f);
        beginProfileLeg(now);
    }

    private void beginProfileLeg(double now) {
        double first = legs.get(legIndex).fractionAt(0) * max;
        metrics.begin(first, velocity());
        beginLeg(now, first);
    }

    private void validate(double now) {
        TargetProfile.Leg leg = legs.get(legIndex);
        status = String.format(Locale.US, "Validating %s %d/%d (P=%.4f F=%.3f)",
                leg.ramp ? "ramp" : "hold", legIndex + 1, legs.size(), p, f);

        double t = now - start;
        if (leg.ramp) {
            target = leg.fractionAt(t) * max;
            motor.setVelocity(target);
            // Ignore the first part of a ramp, where the flywheel is still picking up the slope
            if (t >= 0.3 * leg.seconds) {
                rampSumAbs += Math.abs(velocity() - target) / max;
                rampCount++;
            }
            if (t < leg.seconds) return;
        } else {
            metrics.add(velocity());
            // The first scored hold is the spin-up from idle, which is the thing a shooter needs to be quick at
            if (legIndex == 1 && riseTime < 0 && metrics.smoothed() >= riseThreshold()) riseTime = now - start;
            if (legIndex == 1 && settleTime < 0 && Math.abs(metrics.smoothed() / target - 1) <= cfg.settleBand) settleTime = now - start;
            if (!legDone(now)) return;
            noteIfSlow();
            if (leg.scored) {
                // Only a real step says anything about overshoot; a hold that follows a ramp barely moves
                if (metrics.stepSize() >= 0.1 * max) {
                    // The pass is judged on the spin-up to the operating speed, which is what gets shot. The other
                    // steps are reported if they overshoot.
                    if (legIndex == 1) {
                        valMaxOvershoot = Math.max(valMaxOvershoot, metrics.overshootOfTarget());
                        valRinging |= metrics.flips() >= cfg.maxFlips;
                    } else {
                        valOtherOvershoot = Math.max(valOtherOvershoot, metrics.overshootOfTarget());
                    }
                }
                double rel = (window.mean() - target) / target;
                holdSumAbs += Math.abs(rel);
                holdSumSigned += rel;
                holdCount++;
                if (holdReport.length() > 0) holdReport.append("  ");
                holdReport.append(String.format(Locale.US, "%.0f:%+.1f%%", target, rel * 100));
            }
        }

        if (++legIndex < legs.size()) {
            beginProfileLeg(now);
            return;
        }

        lastMeanError = holdSumAbs / holdCount;
        lastRampError = rampSumAbs / rampCount;
        double bias = holdSumSigned / holdCount;
        if (valOtherOvershoot > cfg.maxOvershoot) {
            notes.append(String.format(Locale.US, "A step to another speed overshot by %.1f%% of its target; ", valOtherOvershoot * 100));
        }
        boolean errorOk = lastMeanError <= cfg.validateTolerance;
        boolean overshootOk = valMaxOvershoot <= cfg.maxOvershoot && !valRinging;
        // I is what removes a steady error, so a leftover one is a reason to tune it even inside the tolerance.
        // Otherwise P has to be raised until it does the job, which makes the motor chatter.
        boolean rampOk = lastRampError <= cfg.rampTolerance;
        boolean needI = !iTuned && (!errorOk || !rampOk || Math.abs(bias) > cfg.iBiasTrigger);
        if (errorOk && overshootOk && rampOk && !needI) {
            finish(true);
        } else if (encoderInverted) {
            // Only F can be adjusted, so correct it for any average bias once
            if (!fCorrected && Math.abs(bias) > cfg.biasCorrectionThreshold) {
                f /= clamp(1 + bias, 0.5, 2.0);
                fCorrected = true;
                notes.append(String.format(Locale.US, "F corrected for %.1f%% bias; ", bias * 100));
                startValidate(now);
            } else {
                notes.append(offsetNote(bias));
                finish(errorOk && overshootOk);
            }
        } else if ((needI || !overshootOk && !dTuned) && budgetLow(now)) {
            notes.append("Ran out of time before I or D could be tuned; ");
            finish(errorOk && overshootOk);
        } else if (needI) {
            iTuned = true;
            notes.append(String.format(Locale.US, "Hold error %+.1f%% and ramp error %.1f%% after P, so I was tuned; ", bias * 100, lastRampError * 100));
            startSearch(now, Param.I);
        } else if (!overshootOk && !dTuned) {
            // Overshoot left over is what D is for
            dTuned = true;
            notes.append(String.format(Locale.US, "Overshoot %.1f%% remains, so D was tuned; ", valMaxOvershoot * 100));
            startSearch(now, Param.D);
        } else {
            if (!errorOk) notes.append(offsetNote(bias));
            if (!rampOk) notes.append(String.format(Locale.US, "Ramp error %.1f%% of top speed is above the %.1f%% allowed; ", lastRampError * 100, cfg.rampTolerance * 100));
            if (!overshootOk) notes.append(String.format(Locale.US, "Overshoot %.1f%% of the target remains; ", valMaxOvershoot * 100));
            finish(false);
        }
    }

    // ---- end states ----

    private void finish(boolean passed) {
        logRow(clock.seconds(), "finished passed=" + passed);
        closeLog();
        motor.stop();
        result = new FlywheelTuneResult(p, i, d, f, motor.getVoltage(), lastMeanError, lastRampError, riseTime, settleTime, valMaxOvershoot,
                passed, encoderInverted, notes.toString().trim(), directions(), holdReport.toString(),
                diagnostics(), searchReport.toString());
        phase = Phase.DONE;
        status = passed ? "Done" : "Done (not converged)";
    }

    private void fail(String reason) {
        logRow(clock.seconds(), "failed " + reason.replace(',', ';'));
        closeLog();
        motor.stop();
        result = new FlywheelTuneResult(p, i, d, f, motor.getVoltage(), Double.NaN, Double.NaN, -1, -1, 0,
                false, encoderInverted, reason, directions(), "", diagnostics(), searchReport.toString());
        phase = Phase.FAILED;
        status = "Failed: " + reason;
    }

    /** Tells a steady offset apart from error that changes sign with speed, which one F cannot fit. */
    private String offsetNote(double bias) {
        if (Math.abs(bias) < 0.5 * lastMeanError) {
            return String.format(Locale.US,
                    "Error averages %.1f%% but runs both fast and slow depending on the speed, so one F cannot fit every speed (feedback P fixes this); ",
                    lastMeanError * 100);
        }
        return String.format(Locale.US, "Steady offset %.1f%% remains; ", bias * 100);
    }

    private String directions() {
        StringBuilder sb = new StringBuilder();
        for (int i = 0; i < motor.getMotorCount(); i++) {
            if (i > 0) sb.append(", ");
            sb.append(motor.getMotorName(i)).append(": ").append(motor.isReversed(i) ? "REVERSE" : "FORWARD");
        }
        return sb.toString();
    }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }
}
