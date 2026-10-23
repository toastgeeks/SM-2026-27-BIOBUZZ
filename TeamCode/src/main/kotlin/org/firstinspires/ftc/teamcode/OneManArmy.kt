package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple

/**
 * One gamepad drives and runs the intake, both flywheels, and the feed.
 * Both flywheels share a direction and a power.
 * flywheel1's encoder is reset at init and reported as ticks and ticks per second.
 */
@TeleOp(name = "One Man Army", group = "Test")
class OneManArmy : LinearOpMode() {
    override fun runOpMode() {
        val drive = BotCentricDrive(hardwareMap)
        val intake = motor(HardwareNames.INTAKE, DcMotorSimple.Direction.REVERSE)
        val flywheel = motor(HardwareNames.FLYWHEEL, DcMotorSimple.Direction.REVERSE)
        resetEncoder(flywheel)
        val flywheel2 = motor(HardwareNames.FLYWHEEL_2, DcMotorSimple.Direction.REVERSE)
        val feed = motor(HardwareNames.FEED, DcMotorSimple.Direction.FORWARD)

        addControls()
        telemetry.update()
        waitForStart()

        while (opModeIsActive()) {
            drive.drive(
                forward = -gamepad1.left_stick_y.toDouble(),
                strafe = gamepad1.left_stick_x.toDouble(),
                turn = gamepad1.right_stick_x.toDouble(),
            )

            val flywheelFromTrigger = gamepad1.left_trigger > 0f
            // Target flywheel RPM is not chosen yet, so this gate stays open.
            val atTargetRpm = true

            val intakePower = when {
                flywheelFromTrigger -> 0.0
                gamepad1.circle -> -1.0
                else -> gamepad1.right_trigger.toDouble()
            }
            val flywheelPower = when {
                gamepad1.square -> -1.0
                else -> gamepad1.left_trigger.toDouble()
            }
            val feedPower = when {
                gamepad1.left_bumper -> -1.0
                gamepad1.right_bumper && atTargetRpm -> 1.0
                else -> 0.0
            }

            intake.power = intakePower
            flywheel.power = flywheelPower
            flywheel2.power = flywheelPower
            feed.power = feedPower

            addControls()
            telemetry.addData("Intake", "%.2f", intakePower)
            telemetry.addData("Flywheel", "%.2f", flywheelPower)
            telemetry.addData("Flywheel1 pos", flywheel.currentPosition)
            telemetry.addData("Flywheel1 ticks/s", "%.0f", flywheel.velocity)
            telemetry.addData("At target RPM", atTargetRpm)
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
        telemetry.addLine("Left stick drives. Right stick turns.")
        telemetry.addLine("RT intake. Circle intake reverse.")
        telemetry.addLine("LT flywheel. Square flywheel reverse. LT ignores intake.")
        telemetry.addLine("RB feed at target RPM. LB feed reverse.")
    }

    private fun resetEncoder(motor: DcMotorEx) {
        motor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        motor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
    }
}
