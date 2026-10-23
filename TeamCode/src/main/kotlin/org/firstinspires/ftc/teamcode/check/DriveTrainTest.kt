package org.firstinspires.ftc.teamcode.check

import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import org.firstinspires.ftc.teamcode.HardwareNames

/**
 * Spins one drive wheel at a time so the d-pad mapping can be checked.
 *
 * Hold a d-pad button to run that wheel forward at 30% power.
 * Front left, front right, and back left are reversed so positive power rolls each wheel forward.
 *
 * D-pad left: front left
 * D-pad up: front right
 * D-pad right: back right
 * D-pad down: back left
 */
@TeleOp(name = "DriveTrainTest", group = "Test")
@Disabled
class DriveTrainTest : LinearOpMode() {
    override fun runOpMode() {
        val frontLeft = motor(HardwareNames.FRONT_LEFT, DcMotorSimple.Direction.REVERSE)
        val frontRight = motor(HardwareNames.RIGHT_FRONT, DcMotorSimple.Direction.REVERSE)
        val backRight = motor(HardwareNames.BACK_RIGHT, DcMotorSimple.Direction.FORWARD)
        val backLeft = motor(HardwareNames.BACK_LEFT, DcMotorSimple.Direction.REVERSE)

        telemetry.addLine("Hold a d-pad button to spin that wheel forward.")
        telemetry.addLine("Left: front left")
        telemetry.addLine("Up: front right")
        telemetry.addLine("Right: back right")
        telemetry.addLine("Down: back left")
        telemetry.update()

        waitForStart()

        while (opModeIsActive()) {
            val frontLeftPower = if (gamepad1.dpad_left) POWER else 0.0
            val frontRightPower = if (gamepad1.dpad_up) POWER else 0.0
            val backRightPower = if (gamepad1.dpad_right) POWER else 0.0
            val backLeftPower = if (gamepad1.dpad_down) POWER else 0.0

            frontLeft.power = frontLeftPower
            frontRight.power = frontRightPower
            backRight.power = backRightPower
            backLeft.power = backLeftPower

            telemetry.addData("Front left (dpad left)", frontLeftPower)
            telemetry.addData("Front right (dpad up)", frontRightPower)
            telemetry.addData("Back right (dpad right)", backRightPower)
            telemetry.addData("Back left (dpad down)", backLeftPower)
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
        const val POWER = 0.3
    }
}
