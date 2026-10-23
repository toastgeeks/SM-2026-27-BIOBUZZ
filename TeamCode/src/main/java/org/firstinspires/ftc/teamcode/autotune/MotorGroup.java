package org.firstinspires.ftc.teamcode.autotune;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

/**
 * One or more motors driving the same mechanism. Every motor gets the same command and the same
 * coefficients; velocity is read from the first motor. Max speed comes from the motor type, so
 * ticks per rev and max RPM never need to be configured by hand.
 */
public class MotorGroup implements TunableMotor {
    private final DcMotorEx[] motors;
    private final String[] names;   // the names from the robot configuration, in the same order as motors
    private final VoltageSensor battery;

    public MotorGroup(HardwareMap hw, String... motorNames) {
        if (motorNames.length == 0) throw new IllegalArgumentException("Need at least one motor");
        motors = new DcMotorEx[motorNames.length];
        names = motorNames.clone();
        for (int i = 0; i < motorNames.length; i++) {
            motors[i] = hw.get(DcMotorEx.class, motorNames[i]);
        }
        battery = hw.voltageSensor.iterator().next();
    }

    @Override
    public int getMotorCount() {
        return motors.length;
    }

    @Override
    public String getMotorName(int index) {
        return names[index];
    }

    @Override
    public void spinMotor(int index, double power) {
        for (int i = 0; i < motors.length; i++) {
            motors[i].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motors[i].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            motors[i].setPower(i == index ? power : 0);
        }
    }

    @Override
    public void reverseMotor(int index) {
        motors[index].setDirection(isReversed(index)
                ? DcMotorSimple.Direction.FORWARD : DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public double getCurrentAmps(int index) {
        return motors[index].getCurrent(CurrentUnit.AMPS);
    }

    @Override
    public double getMotorVelocity(int index) {
        return motors[index].getVelocity();
    }

    @Override
    public void setPrimary(int index) {
        DcMotorEx first = motors[0];
        motors[0] = motors[index];
        motors[index] = first;
        String firstName = names[0];
        names[0] = names[index];
        names[index] = firstName;
    }

    @Override
    public boolean isReversed(int index) {
        return motors[index].getDirection() == DcMotorSimple.Direction.REVERSE;
    }

    @Override
    public double getMaxVelocity() {
        double max = Double.MAX_VALUE;
        for (DcMotorEx m : motors) {
            max = Math.min(max, m.getMotorType().getAchieveableMaxTicksPerSecond());
        }
        return max;
    }

    @Override
    public double getVelocity() {
        return motors[0].getVelocity();
    }

    @Override
    public double getVoltage() {
        return battery.getVoltage();
    }

    @Override
    public void setOpenLoopPower(double power) {
        for (DcMotorEx m : motors) {
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            m.setPower(power);
        }
    }

    @Override
    public void setVelocity(double ticksPerSecond) {
        for (DcMotorEx m : motors) {
            if (m.getMode() != DcMotor.RunMode.RUN_USING_ENCODER) {
                m.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
            m.setVelocity(ticksPerSecond);
        }
    }

    @Override
    public void setPIDF(double p, double i, double d, double f) {
        // Only the first motor closes the loop. The others get F alone, so they add the same forward power.
        // Giving every motor the same P, I and D would let a motor whose feedback works against the
        // flywheel cancel out one whose feedback works.
        for (int k = 0; k < motors.length; k++) {
            motors[k].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            PIDFCoefficients coefficients = k == 0 ? new PIDFCoefficients(p, i, d, f) : new PIDFCoefficients(0, 0, 0, f);
            motors[k].setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, coefficients);
        }
    }

    @Override
    public void stop() {
        setOpenLoopPower(0);
    }
}
