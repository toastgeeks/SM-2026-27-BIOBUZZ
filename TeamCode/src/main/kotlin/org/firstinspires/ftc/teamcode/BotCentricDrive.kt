package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import kotlin.math.abs

/**
 * Robot-centric mecanum drive.
 *
 * Stick commands stay in the robot's frame: forward is ahead of the robot,
 * strafe is to the robot's right, and turn is clockwise.
 * Front left, front right, and back left are reversed so positive power rolls each wheel forward.
 */
class BotCentricDrive(hardwareMap: HardwareMap) {
    private val leftFront = motor(hardwareMap, HardwareNames.FRONT_LEFT, DcMotorSimple.Direction.REVERSE)
    private val rightFront = motor(hardwareMap, HardwareNames.RIGHT_FRONT, DcMotorSimple.Direction.REVERSE)
    private val backLeft = motor(hardwareMap, HardwareNames.BACK_LEFT, DcMotorSimple.Direction.REVERSE)
    private val backRight = motor(hardwareMap, HardwareNames.BACK_RIGHT, DcMotorSimple.Direction.FORWARD)

    /** Positive [forward], strafe right, and clockwise [turn]. Scales powers down so none exceeds 1. */
    fun drive(forward: Double, strafe: Double, turn: Double) {
        var leftFrontPower = forward + strafe + turn
        var backLeftPower = forward - strafe + turn
        var rightFrontPower = forward - strafe - turn
        var backRightPower = forward + strafe - turn

        val max = maxOf(
            1.0,
            abs(leftFrontPower),
            abs(backLeftPower),
            abs(rightFrontPower),
            abs(backRightPower),
        )
        leftFrontPower /= max
        backLeftPower /= max
        rightFrontPower /= max
        backRightPower /= max

        leftFront.power = leftFrontPower
        backLeft.power = backLeftPower
        rightFront.power = rightFrontPower
        backRight.power = backRightPower
    }

    private fun motor(
        hardwareMap: HardwareMap,
        name: String,
        direction: DcMotorSimple.Direction,
    ): DcMotor {
        return hardwareMap.get(DcMotor::class.java, name).apply {
            this.direction = direction
            mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
            zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
            power = 0.0
        }
    }
}
