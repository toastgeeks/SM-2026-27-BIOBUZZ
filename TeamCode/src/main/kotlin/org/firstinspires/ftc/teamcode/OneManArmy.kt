package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import kotlin.math.abs

/**
 * One driver. Left stick drives, right stick turns, triggers and bumpers run the mechanisms.
 */
@TeleOp(name = "One Man Army", group = "Test")
class OneManArmy : LinearOpMode() {
    override fun runOpMode() {
        val leftFront = motor(HardwareNames.FRONT_LEFT, DcMotorSimple.Direction.REVERSE)
        val rightFront = motor(HardwareNames.RIGHT_FRONT, DcMotorSimple.Direction.FORWARD)
        val backLeft = motor(HardwareNames.BACK_LEFT, DcMotorSimple.Direction.REVERSE)
        val backRight = motor(HardwareNames.BACK_RIGHT, DcMotorSimple.Direction.FORWARD)
        val intake = motor(HardwareNames.INTAKE, DcMotorSimple.Direction.REVERSE)
        val flywheel = motor(HardwareNames.FLYWHEEL, DcMotorSimple.Direction.FORWARD)
        val flywheel2 = motor(HardwareNames.FLYWHEEL_2, DcMotorSimple.Direction.REVERSE)
        val feed = motor(HardwareNames.FEED, DcMotorSimple.Direction.FORWARD)

        telemetry.addLine("Left stick drives, right stick turns.")
        telemetry.addLine("LT positive intake, LB negative intake, RT flywheel, RB feed.")
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

            val intakePower = if (gamepad1.left_bumper) -1.0 else gamepad1.left_trigger.toDouble()
            val flywheelPower = gamepad1.right_trigger.toDouble()
            val feedPower = if (gamepad1.right_bumper) 1.0 else 0.0

            intake.power = intakePower
            flywheel.power = flywheelPower
            flywheel2.power = flywheelPower
            feed.power = feedPower

            telemetry.addData("Intake", intakePower)
            telemetry.addData("Flywheel", flywheelPower)
            telemetry.addData("Feed", feedPower)
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
