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

    /** Starts a local, d-pad-adjustable setpoint. LT runs it; square reverses with priority. */
    protected open val initialShooterVelocity: Double? = null

    /** Uses shared dashboard PIDF with either a local or shared velocity target. */
    protected open val useShooterVelocityControl: Boolean
        get() = initialShooterVelocity != null

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
        val initialShooterTarget = initialShooterVelocity
        var selectedShooterTarget = initialShooterTarget ?: Shooter.TARGET

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
        if (initialShooterTarget != null) {
            telemetry.addLine("LT > 0.1: selected velocity. Square: reverse (priority). Neither: 0. LT ignores intake.")
        } else {
            telemetry.addLine("LT flywheel. Square flywheel reverse. LT ignores intake.")
        }
        telemetry.addLine("RB feed. LB feed reverse.")
        if (useShooterVelocityControl) {
            telemetry.addLine("D-pad up/down adjusts shooter target by ${Shooter.TARGET_STEP} ticks/s.")
            if (initialShooterTarget == null) {
                telemetry.addLine("Dashboard config: Shooter PIDF_*, TARGET and TARGET_STEP.")
            } else {
                telemetry.addLine("Dashboard config: Shooter PIDF_* and TARGET_STEP.")
            }
            telemetry.addData("Shooter target (ticks/s)", 0.0)
            telemetry.addData("Shooter selected (ticks/s)", selectedShooterTarget)
            addShooterPIDFTelemetry()
        }
        telemetry.update()

        if (useShooterVelocityControl) {
            while (opModeInInit()) {
                updateShooterPIDF()
                if (initialShooterTarget == null) selectedShooterTarget = Shooter.TARGET
                telemetry.addData("Shooter target (ticks/s)", 0.0)
                telemetry.addData("Shooter selected (ticks/s)", selectedShooterTarget)
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
                val change = (if (up && !targetUpWasPressed) 1 else 0) -
                    (if (down && !targetDownWasPressed) 1 else 0)
                if (initialShooterTarget == null) {
                    if (change != 0) Shooter.TARGET += change * Shooter.TARGET_STEP
                    selectedShooterTarget = Shooter.TARGET
                } else {
                    selectedShooterTarget = (selectedShooterTarget + change * Shooter.TARGET_STEP)
                        .coerceAtLeast(0.0)
                }
                targetUpWasPressed = up
                targetDownWasPressed = down

                updateShooterPIDF()
            }
            val shooterTarget = when {
                manipulator.square -> -selectedShooterTarget
                manipulator.left_trigger > 0.1f -> selectedShooterTarget
                else -> 0.0
            }

            when {
                manipulator.left_trigger > 0f -> intake.off()
                manipulator.circle -> intake.reverse()
                manipulator.right_trigger > 0.1f -> intake.on()
                else -> intake.off()
            }.schedule()
            if (useShooterVelocityControl) {
                shooter.runVelocity(shooterTarget)
            } else {
                when {
                    manipulator.square -> shooter.reverse()
                    manipulator.left_trigger > 0.1f -> shooter.on()
                    else -> shooter.off()
                }
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
                telemetry.addData("Shooter selected (ticks/s)", selectedShooterTarget)
                telemetry.addData("Flywheel2 ticks/s", "%.0f", shooter.flywheel2Velocity)
                addShooterPIDFTelemetry()
            }
            telemetry.update()
        }
    }
}
