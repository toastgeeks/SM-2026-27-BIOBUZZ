package org.firstinspires.ftc.teamcode.auto

import com.pedropathing.ivy.Scheduler
import com.pedropathing.ivy.groups.Groups.sequential
import com.pedropathing.ivy.pedro.PedroCommands.follow
import com.pedropathing.ivy.pedro.PedroCommands.hold
import com.pedropathing.math.Pose
import com.pedropathing.paths.AtomicPath
import com.pedropathing.paths.curves.Line
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.pedro.Constants

/**
 * Drives forward 24 inches with Pedro, then holds the end pose.
 */
@Autonomous(name = "Auto", group = "Auto")
class Auto : LinearOpMode() {
    override fun runOpMode() {
        Scheduler.reset()
        val follower = Constants.create(hardwareMap)
        val start = Pose(0.0, 0.0, 0.0)
        val end = Pose(24.0, 0.0, 0.0)
        follower.setPose(start)

        val forward = AtomicPath(Line(start, end)).linear(start, end)
        val routine = sequential(
            follow(follower, forward),
            hold(follower, end),
        )

        telemetry.addLine("Drives forward 24 in, then holds.")
        telemetry.update()
        waitForStart()

        routine.schedule()
        while (opModeIsActive()) {
            follower.update()
            Scheduler.execute()

            val pose = follower.pose()
            telemetry.addData("X (in)", "%.2f", pose.x())
            telemetry.addData("Y (in)", "%.2f", pose.y())
            telemetry.addData("Heading (deg)", "%.1f", Math.toDegrees(pose.heading()))
            telemetry.update()
        }
    }
}
