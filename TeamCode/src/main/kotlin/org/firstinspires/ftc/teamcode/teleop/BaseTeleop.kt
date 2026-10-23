package org.firstinspires.ftc.teamcode.teleop

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

    override fun runOpMode() {
        Scheduler.reset()
        val follower = Constants.create(hardwareMap)
        follower.setPose(Pose(0.0, 0.0, 0.0))
        val intake = Intake(hardwareMap)
        val shooter = Shooter(hardwareMap)
        val feed = Feed(hardwareMap)

        telemetry.addLine("GP1 left stick drives. Right stick turns.")
        telemetry.addLine("GP1 triangle sets heading to 0.")
        telemetry.addLine("RT intake. Circle intake reverse.")
        telemetry.addLine("LT flywheel. Square flywheel reverse. LT ignores intake.")
        telemetry.addLine("RB feed. LB feed reverse.")
        telemetry.update()
        waitForStart()

        var headingResetWasPressed = false

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

            when {
                manipulator.left_trigger > 0f -> intake.off()
                manipulator.circle -> intake.reverse()
                manipulator.right_trigger > 0.1f -> intake.on()
                else -> intake.off()
            }.schedule()
            when {
                manipulator.square -> shooter.reverse()
                manipulator.left_trigger > 0.1f -> shooter.on()
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
            telemetry.update()
        }
    }
}
