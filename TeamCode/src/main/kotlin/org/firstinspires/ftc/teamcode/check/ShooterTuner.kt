package org.firstinspires.ftc.teamcode.check

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.pedropathing.ivy.Scheduler
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.subsystems.Shooter
import org.firstinspires.ftc.teamcode.subsystems.Shooter.Companion.TARGET
import org.firstinspires.ftc.teamcode.subsystems.Shooter.Companion.TARGET_STEP

/** Starts both flywheels at 800 ticks/s. Gamepad 1 d-pad changes the target once per press. */
@TeleOp(name = "ShooterTuner", group = "Tuning")
class ShooterTuner : LinearOpMode() {
    override fun runOpMode() {
        Scheduler.reset()
        telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)
        var pidf = Shooter.PIDF_COEFFICIENTS
        val shooter = Shooter(hardwareMap, pidf)
        var upWasPressed = false
        var downWasPressed = false

        telemetry.addLine("Start spins both flywheels at the target velocity.")
        telemetry.addLine("GP1 d-pad up/down: target +/-$TARGET_STEP ticks/s per press.")
        telemetry.addLine("Dashboard config: Shooter PIDF_*, TARGET and TARGET_STEP.")
        telemetry.addData("Target (ticks/s)", TARGET)
        telemetry.addData("PIDF", "P=%.1f I=%.1f D=%.1f F=%.1f", pidf.p, pidf.i, pidf.d, pidf.f)
        telemetry.update()

        try {
            waitForStart()

            while (opModeIsActive()) {
                val up = gamepad1.dpad_up
                val down = gamepad1.dpad_down
                if (up && !upWasPressed) TARGET += TARGET_STEP
                if (down && !downWasPressed) TARGET -= TARGET_STEP
                upWasPressed = up
                downWasPressed = down

                val updatedPIDF = Shooter.PIDF_COEFFICIENTS
                if (updatedPIDF.p != pidf.p || updatedPIDF.i != pidf.i ||
                    updatedPIDF.d != pidf.d || updatedPIDF.f != pidf.f
                ) {
                    shooter.setPIDF(updatedPIDF)
                    pidf = updatedPIDF
                }
                val target = TARGET
                shooter.runVelocity(target).schedule()
                Scheduler.execute()

                val velocity1 = shooter.velocity
                val velocity2 = shooter.flywheel2Velocity
                telemetry.addData("Target (ticks/s)", target)
                telemetry.addData("Flywheel1 (ticks/s)", "%.0f", velocity1)
                telemetry.addData("Flywheel2 (ticks/s)", "%.0f", velocity2)
                telemetry.addData("Flywheel1 error (ticks/s)", "%.0f", target - velocity1)
                telemetry.addData("Flywheel2 error (ticks/s)", "%.0f", target - velocity2)
                telemetry.addData("PIDF", "P=%.1f I=%.1f D=%.1f F=%.1f", pidf.p, pidf.i, pidf.d, pidf.f)
                telemetry.addLine("GP1 d-pad up/down: target +/-$TARGET_STEP ticks/s per press.")
                telemetry.update()
            }
        } finally {
            shooter.off().schedule()
            Scheduler.execute()
            Scheduler.reset()
        }
    }
}
