package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple

/**
 * Drives forward for one second, then stops.
 */
@Autonomous(name = "Auto", group = "Auto")
class Auto : LinearOpMode() {
    override fun runOpMode() {
        val leftFront = motor(HardwareNames.FRONT_LEFT, DcMotorSimple.Direction.REVERSE)
        val rightFront = motor(HardwareNames.RIGHT_FRONT, DcMotorSimple.Direction.FORWARD)
        val backLeft = motor(HardwareNames.BACK_LEFT, DcMotorSimple.Direction.REVERSE)
        val backRight = motor(HardwareNames.BACK_RIGHT, DcMotorSimple.Direction.FORWARD)
        val drive = listOf(leftFront, rightFront, backLeft, backRight)

        telemetry.addLine("Drives forward for 1 second at half power.")
        telemetry.update()
        waitForStart()

        drive.forEach { it.power = POWER }
        sleep(1000)
        drive.forEach { it.power = 0.0 }
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
        const val POWER = 0.5
    }
}
