package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import kotlin.math.abs

/**
 * Robot-centric mecanum drive. Left stick drives, right stick turns.
 * Left motors are reversed so positive power rolls the robot forward.
 */
@TeleOp(name = "Check: Drive", group = "Check")
class DriveCheck : LinearOpMode() {
    override fun runOpMode() {
        val leftFront = motor(HardwareNames.FRONT_LEFT, DcMotorSimple.Direction.REVERSE)
        val rightFront = motor(HardwareNames.RIGHT_FRONT, DcMotorSimple.Direction.FORWARD)
        val backLeft = motor(HardwareNames.BACK_LEFT, DcMotorSimple.Direction.REVERSE)
        val backRight = motor(HardwareNames.BACK_RIGHT, DcMotorSimple.Direction.FORWARD)

        telemetry.addLine("Left stick: drive. Right stick X: turn.")
        telemetry.update()
        waitForStart()

        while (opModeIsActive()) {
            val forward = -gamepad1.left_stick_y.toDouble()
            val strafe = gamepad1.left_stick_x.toDouble()
            val turn = gamepad1.right_stick_x.toDouble()

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

            telemetry.addData("Left front", leftFrontPower)
            telemetry.addData("Right front", rightFrontPower)
            telemetry.addData("Back left", backLeftPower)
            telemetry.addData("Back right", backRightPower)
            telemetry.update()
        }
    }

    private fun motor(name: String, direction: DcMotorSimple.Direction): DcMotor {
        return hardwareMap.get(DcMotor::class.java, name).apply {
            this.direction = direction
            mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
            zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
            power = 0.0
        }
    }
}
