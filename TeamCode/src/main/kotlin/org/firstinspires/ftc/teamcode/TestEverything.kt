package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import kotlin.math.abs

/**
 * Gamepad 1 drives. Gamepad 2 runs the intake, both flywheels, and the feed.
 * Each of those can run positive or negative. Both flywheels share a direction and a power.
 * flywheel1's encoder is reset at init and reported as ticks and ticks per second.
 */
@TeleOp(name = "Test Everything", group = "Test")
class TestEverything : LinearOpMode() {
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

            val intakePower = if (gamepad2.left_bumper) -1.0 else gamepad2.left_trigger.toDouble()
            val flywheelPower = -gamepad2.right_stick_y.toDouble()
            val feedPower = if (gamepad2.right_bumper) -1.0 else gamepad2.right_trigger.toDouble()

            intake.power = intakePower
            flywheel.power = flywheelPower
            flywheel2.power = flywheelPower
            feed.power = feedPower

            addControls()
            telemetry.addData("Intake", "%.2f", intakePower)
            telemetry.addData("Flywheel", "%.2f", flywheelPower)
            telemetry.addData("Flywheel1 pos", flywheel.currentPosition)
            telemetry.addData("Flywheel1 ticks/s", "%.0f", flywheel.velocity)
            telemetry.addData("Feed", "%.2f", feedPower)
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
        telemetry.addLine("GP1 left stick drives. Right stick turns.")
        telemetry.addLine("GP2 LT intake positive. LB intake negative.")
        telemetry.addLine("GP2 right stick Y flywheels. Up positive, down negative.")
        telemetry.addLine("GP2 RT feed positive. RB feed negative.")
    }

    private fun resetEncoder(motor: DcMotorEx) {
        motor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        motor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
    }
}
