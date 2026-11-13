package org.firstinspires.ftc.teamcode.auto

import com.pedropathing.drivetrain.DrivePowers
import com.pedropathing.revhub.drivetrains.Mecanum
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.pedro.Constants

/**
 * Drives forward for one second, strafes right for half a second, then stops.
 */
@Autonomous(name = "Auto", group = "Auto")
class Auto : LinearOpMode() {
    override fun runOpMode() {
        val drivetrain = Mecanum(hardwareMap, Constants.drivetrainConfig)
        drivetrain.stop(true)
        val timer = ElapsedTime()

        telemetry.addLine("Forward 1 s, strafe right 0.5 s, then stop at 30% power.")
        telemetry.update()

        try {
            waitForStart()
            timer.reset()

            while (opModeIsActive()) {
                val elapsed = timer.seconds()
                val phase = when {
                    elapsed < FORWARD_SECONDS -> {
                        drivetrain.drive(DrivePowers(DRIVE_POWER, 0.0, 0.0), true)
                        "Forward"
                    }
                    elapsed < FORWARD_SECONDS + STRAFE_SECONDS -> {
                        // Pedro uses positive strafe for left, so right is negative.
                        drivetrain.drive(DrivePowers(0.0, -DRIVE_POWER, 0.0), true)
                        "Strafe right"
                    }
                    else -> {
                        drivetrain.stop(true)
                        "Complete"
                    }
                }

                telemetry.addData("Phase", phase)
                telemetry.addData("Elapsed (s)", "%.2f", elapsed)
                telemetry.update()
                if (elapsed >= FORWARD_SECONDS + STRAFE_SECONDS) break
                idle()
            }
        } finally {
            drivetrain.stop(true)
        }
    }

    private companion object {
        const val DRIVE_POWER = 0.3
        const val FORWARD_SECONDS = 1.0
        const val STRAFE_SECONDS = 0.5
    }
}
