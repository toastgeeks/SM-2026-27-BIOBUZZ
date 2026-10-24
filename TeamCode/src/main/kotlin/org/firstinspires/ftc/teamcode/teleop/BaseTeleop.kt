package org.firstinspires.ftc.teamcode.teleop

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.pedropathing.follower.ManualDrive
import com.pedropathing.ivy.Scheduler
import com.pedropathing.math.Pose
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.Gamepad
import org.firstinspires.ftc.teamcode.pedro.Constants
import org.firstinspires.ftc.teamcode.subsystems.Feed
import org.firstinspires.ftc.teamcode.subsystems.Intake
import org.firstinspires.ftc.teamcode.subsystems.Shooter

/**
 * Gamepad 1 drives field-centric through the Pedro follower. [manipulator] runs the
 * intake, flywheels, and feed as Ivy commands.
 *
 * Triangle on gamepad 1 sets the current heading to zero. Point the robot, then press it.
 * Manipulator: RT intake, circle intake reverse. LT flywheel, square flywheel reverse.
 * LT ignores intake. RB feed, LB feed reverse.
 */
abstract class BaseTeleop : LinearOpMode() {
    /** The gamepad that runs the mechanisms. Read after the OpMode is initialized. */
    protected abstract val manipulator: Gamepad

    /** Opts a TeleOp into shared dashboard PIDF, velocity target, and d-pad adjustments. */
    protected open val useShooterVelocityControl = false

    override fun runOpMode() {
        Scheduler.reset()
        if (useShooterVelocityControl) {
            telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)
        }
        val follower = Constants.create(hardwareMap)
        follower.setPose(Pose(0.0, 0.0, 0.0))
        val intake = Intake(hardwareMap)
        var shooterPIDF = if (useShooterVelocityControl) Shooter.PIDF_COEFFICIENTS else null
        val shooter = Shooter(hardwareMap, shooterPIDF)
        val feed = Feed(hardwareMap)

        fun updateShooterPIDF() {
            val appliedPIDF = shooterPIDF ?: return
            val updatedPIDF = Shooter.PIDF_COEFFICIENTS
            if (updatedPIDF.p != appliedPIDF.p || updatedPIDF.i != appliedPIDF.i ||
                updatedPIDF.d != appliedPIDF.d || updatedPIDF.f != appliedPIDF.f
            ) {
                shooter.setPIDF(updatedPIDF)
                shooterPIDF = updatedPIDF
            }
        }

        fun addShooterPIDFTelemetry() {
            val appliedPIDF = shooterPIDF ?: return
            telemetry.addData("Shooter P", appliedPIDF.p)
            telemetry.addData("Shooter I", appliedPIDF.i)
            telemetry.addData("Shooter D", appliedPIDF.d)
            telemetry.addData("Shooter F", appliedPIDF.f)
        }

        telemetry.addLine("GP1 left stick drives. Right stick turns.")
        telemetry.addLine("GP1 triangle sets heading to 0.")
        telemetry.addLine("RT intake. Circle intake reverse.")
        telemetry.addLine("LT flywheel. Square flywheel reverse. LT ignores intake.")
        telemetry.addLine("RB feed. LB feed reverse.")
        if (useShooterVelocityControl) {
            telemetry.addLine("D-pad up/down adjusts shooter target by ${Shooter.TARGET_STEP} ticks/s.")
            telemetry.addLine("Dashboard config: Shooter PIDF_*, TARGET and TARGET_STEP.")
            telemetry.addData("Shooter target (ticks/s)", Shooter.TARGET)
            addShooterPIDFTelemetry()
        }
        telemetry.update()

        if (useShooterVelocityControl) {
            while (opModeInInit()) {
                updateShooterPIDF()
                telemetry.addData("Shooter target (ticks/s)", Shooter.TARGET)
                addShooterPIDFTelemetry()
                telemetry.update()
                idle()
            }
        }
        waitForStart()
        if (isStopRequested) return

        var headingResetWasPressed = false
        var targetUpWasPressed = false
        var targetDownWasPressed = false

        while (opModeIsActive()) {
            val resetHeading = gamepad1.triangle
            if (resetHeading && !headingResetWasPressed) {
                follower.setHeading(0.0)
            }
            headingResetWasPressed = resetHeading

            // Pedro is left-positive and counterclockwise-positive, so the sticks are negated.
            follower.manual(
                ManualDrive.fieldCentric(
                    -gamepad1.left_stick_y.toDouble(),
                    -gamepad1.left_stick_x.toDouble(),
                    -gamepad1.right_stick_x.toDouble(),
                    follower.pose().heading(),
                ),
            )
            follower.update()

            if (useShooterVelocityControl) {
                val up = manipulator.dpad_up
                val down = manipulator.dpad_down
                if (up && !targetUpWasPressed) Shooter.TARGET += Shooter.TARGET_STEP
                if (down && !targetDownWasPressed) Shooter.TARGET -= Shooter.TARGET_STEP
                targetUpWasPressed = up
                targetDownWasPressed = down

                updateShooterPIDF()
            }
            val shooterTarget = Shooter.TARGET

            when {
                manipulator.left_trigger > 0f -> intake.off()
                manipulator.circle -> intake.reverse()
                manipulator.right_trigger > 0.1f -> intake.on()
                else -> intake.off()
            }.schedule()
            when {
                manipulator.square -> if (useShooterVelocityControl) {
                    shooter.runVelocity(-shooterTarget)
                } else {
                    shooter.reverse()
                }
                manipulator.left_trigger > 0.1f -> if (useShooterVelocityControl) {
                    shooter.runVelocity(shooterTarget)
                } else {
                    shooter.on()
                }
                else -> shooter.off()
            }.schedule()
            when {
                manipulator.left_bumper -> feed.reverse()
                manipulator.right_bumper -> feed.on()
                else -> feed.off()
            }.schedule()

            Scheduler.execute()

            val pose = follower.pose()
            telemetry.addData("X (in)", "%.2f", pose.x())
            telemetry.addData("Y (in)", "%.2f", pose.y())
            telemetry.addData("Heading (deg)", "%.1f", Math.toDegrees(pose.heading()))
            telemetry.addData("Flywheel1 pos", shooter.position)
            telemetry.addData("Flywheel1 ticks/s", "%.0f", shooter.velocity)
            if (useShooterVelocityControl) {
                telemetry.addData("Shooter target (ticks/s)", shooterTarget)
                telemetry.addData("Flywheel2 ticks/s", "%.0f", shooter.flywheel2Velocity)
                addShooterPIDFTelemetry()
            }
            telemetry.update()
        }
    }
}
