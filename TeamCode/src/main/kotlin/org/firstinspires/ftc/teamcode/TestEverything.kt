package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import kotlin.math.abs

/**
 * Gamepad 1 drives. Gamepad 2 runs the intake, flywheel, and feed.
 */
@TeleOp(name = "Test Everything", group = "Test")
class TestEverything : LinearOpMode() {
    override fun runOpMode() {
        val leftFront = motor(HardwareNames.FRONT_LEFT, DcMotorSimple.Direction.REVERSE)
        val rightFront = motor(HardwareNames.RIGHT_FRONT, DcMotorSimple.Direction.FORWARD)
        val backLeft = motor(HardwareNames.BACK_LEFT, DcMotorSimple.Direction.REVERSE)
        val backRight = motor(HardwareNames.BACK_RIGHT, DcMotorSimple.Direction.FORWARD)
        val intake = motor(HardwareNames.INTAKE, DcMotorSimple.Direction.REVERSE)
        val flywheel = motor(HardwareNames.FLYWHEEL, DcMotorSimple.Direction.FORWARD)
        val flywheel2 = motor(HardwareNames.FLYWHEEL_2, DcMotorSimple.Direction.REVERSE)
        val feed = motor(HardwareNames.FEED, DcMotorSimple.Direction.FORWARD)

        telemetry.addLine("Gamepad 1: left stick drives, right stick turns.")
        telemetry.addLine("Gamepad 2: LB positive intake, LT negative intake, Dpad up/down flywheel ±0.05, RT flywheel, RB feed forward, X feed reverse.")
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

            val intakePower = if (gamepad2.left_bumper) 1.0 else -gamepad2.left_trigger.toDouble()
            val feedPower = when {
                gamepad2.right_bumper -> 1.0
                gamepad2.cross -> -1.0
                else -> 0.0
            }

            val flywheelUp = gamepad2.dpad_up
            val flywheelDown = gamepad2.dpad_down
            if (flywheelUp && !flywheelUpWasPressed) {
                flywheelPower = (flywheelPower + FLYWHEEL_STEP).coerceAtMost(1.0)
            }
            if (flywheelDown && !flywheelDownWasPressed) {
                flywheelPower = (flywheelPower - FLYWHEEL_STEP).coerceAtLeast(0.0)
            }
            flywheelUpWasPressed = flywheelUp
            flywheelDownWasPressed = flywheelDown
            val flywheelOutput = if (gamepad2.right_trigger > 0f) flywheelPower else 0.0

            intake.power = intakePower
            flywheel.power = flywheelOutput
            flywheel2.power = flywheelOutput
            feed.power = feedPower

            telemetry.addData("Intake", intakePower)
            telemetry.addData("Flywheel", "%.2f", flywheelPower)
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

    private companion object {
        const val FLYWHEEL_INITIAL = 0.8
        const val FLYWHEEL_STEP = 0.05
    }
}
