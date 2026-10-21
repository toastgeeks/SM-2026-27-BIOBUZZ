package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
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
        val flywheel = motor(HardwareNames.FLYWHEEL, DcMotorSimple.Direction.REVERSE)
        resetEncoder(flywheel)
        val flywheel2 = motor(HardwareNames.FLYWHEEL_2, DcMotorSimple.Direction.REVERSE)
        val feed = motor(HardwareNames.FEED, DcMotorSimple.Direction.FORWARD)

        addControls()
        telemetry.update()
        waitForStart()

        var flywheelPower = FLYWHEEL_INITIAL
        var flywheelUpWasPressed = false
        var flywheelDownWasPressed = false

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
            val feedPower = if (gamepad1.right_bumper) 1.0 else 0.0

            val flywheelUp = gamepad1.dpad_up
            val flywheelDown = gamepad1.dpad_down
            if (flywheelUp && !flywheelUpWasPressed) {
                flywheelPower = (flywheelPower + FLYWHEEL_STEP).coerceAtMost(1.0)
            }
            if (flywheelDown && !flywheelDownWasPressed) {
                flywheelPower = (flywheelPower - FLYWHEEL_STEP).coerceAtLeast(0.0)
            }
            flywheelUpWasPressed = flywheelUp
            flywheelDownWasPressed = flywheelDown
            val flywheelOutput = if (gamepad1.right_trigger > 0f) flywheelPower else 0.0

            intake.power = intakePower
            flywheel.power = flywheelOutput
            flywheel2.power = flywheelOutput
            feed.power = feedPower

            addControls()
            telemetry.addData("Intake", intakePower)
            telemetry.addData("Flywheel", "%.2f", flywheelPower)
            telemetry.addData("Flywheel1 pos", flywheel.currentPosition)
            telemetry.addData("Flywheel1 ticks/s", "%.0f", flywheel.velocity)
            telemetry.addData("Feed", feedPower)
            telemetry.update()
        }
    }

    private fun motor(name: String, direction: DcMotorSimple.Direction): DcMotorEx {
        return hardwareMap.get(DcMotorEx::class.java, name).apply {
            this.direction = direction
            mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
            zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
            power = 0.0
        }
    }

    private fun addControls() {
        telemetry.addLine("Left stick drives. Right stick turns.")
        telemetry.addLine("LT intake positive. LB intake negative.")
        telemetry.addLine("Dpad up/down flywheel ±0.05. RT runs flywheel.")
        telemetry.addLine("RB feed positive.")
    }

    private fun resetEncoder(motor: DcMotorEx) {
        motor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        motor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
    }

    private companion object {
        const val FLYWHEEL_INITIAL = 0.8
        const val FLYWHEEL_STEP = 0.05
    }
}
