package org.firstinspires.ftc.teamcode.autotune;

/**
 * What the tuner needs from a velocity-controlled mechanism. {@link MotorGroup} implements this
 * for real motors; a simulated flywheel can implement it for testing the tuning logic off-robot.
 */
public interface TunableMotor {
    /** Fastest speed the tuner should treat as 100%, in ticks per second. */
    double getMaxVelocity();

    /** Measured velocity in ticks per second. Positive when commanded velocity is positive. */
    double getVelocity();

    double getVoltage();

    /** Raw power with no velocity control. Only used by the wiring check. */
    void setOpenLoopPower(double power);

    /** Closed-loop velocity target in ticks per second. */
    void setVelocity(double ticksPerSecond);

    void setPIDF(double p, double i, double d, double f);

    void stop();

    // ---- Per-motor access, used by the direction check ----

    int getMotorCount();

    String getMotorName(int index);

    /** Spins only this motor (open loop); every other motor is set to zero power and floats. */
    void spinMotor(int index, double power);

    /** Flips this motor's direction. The SDK flips its encoder reading with it. */
    void reverseMotor(int index);

    boolean isReversed(int index);

    /** Current drawn by one motor, in amps. It follows the torque, so it is the closest thing to the power being applied. */
    double getCurrentAmps(int index);

    /** The raw velocity of one motor, for checking which way its own encoder counts. */
    double getMotorVelocity(int index);

    /** Makes this motor the first one, the one that is measured and closes the loop. */
    void setPrimary(int index);
}
