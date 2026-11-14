package org.firstinspires.ftc.teamcode.auto

import com.pedropathing.api.Paths.curve
import com.pedropathing.api.Paths.line
import com.pedropathing.ivy.Command
import com.pedropathing.ivy.Scheduler
import com.pedropathing.ivy.groups.Groups.parallel
import com.pedropathing.math.Pose
import com.pedropathing.math.Vector2D
import com.pedropathing.paths.Path
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.auto.route.Leg
import org.firstinspires.ftc.teamcode.auto.route.LegHeading
import org.firstinspires.ftc.teamcode.auto.route.Route
import org.firstinspires.ftc.teamcode.auto.route.RouteCommands
import org.firstinspires.ftc.teamcode.auto.route.RouteDrive
import org.firstinspires.ftc.teamcode.auto.route.RouteException
import org.firstinspires.ftc.teamcode.auto.route.RouteMechanisms
import org.firstinspires.ftc.teamcode.auto.route.RoutePlanner
import org.firstinspires.ftc.teamcode.auto.route.RoutePose
import org.firstinspires.ftc.teamcode.auto.route.RouteSpec
import org.firstinspires.ftc.teamcode.pedro.Constants
import org.firstinspires.ftc.teamcode.subsystems.Feed
import org.firstinspires.ftc.teamcode.subsystems.Intake
import org.firstinspires.ftc.teamcode.subsystems.Shooter

/**
 * Auto that runs a route built with `route { ... }`. See auto/route/README.md.
 *
 * [RoutePlanner] checks the route, [RouteCommands] turns it into one Ivy command, and this class
 * connects that command to Pedro and the subsystems and runs the scheduler.
 *
 * INIT shows the route and any warnings on the Driver Station. A route that can't be planned shows
 * the error and the robot does not move.
 */
abstract class RouteAuto(internal val spec: RouteSpec) : LinearOpMode() {

    override fun runOpMode() {
        Scheduler.reset()
        val route = try {
            RoutePlanner.plan(spec)
        } catch (e: RouteException) {
            showError(e.message ?: "Route error")
            return
        }

        val follower = Constants.create(hardwareMap)
        val shooter = Shooter(hardwareMap, Shooter.PIDF_COEFFICIENTS)
        val feed = Feed(hardwareMap)
        val intake = Intake(hardwareMap)
        val paths = route.legs.associateWith(::toPath)

        val drive = object : RouteDrive {
            override fun follow(leg: Leg) = follower.follow(paths.getValue(leg))
            override fun atLegEnd() = follower.atParametricEnd()
            override fun hold(pose: RoutePose) = follower.hold(pose.toPedro())
            override fun stop() = follower.stop()
        }
        val mechanisms = object : RouteMechanisms {
            override val flywheelVelocity get() = shooter.velocity
            override fun spinUp(velocity: Double) = shooter.runVelocity(velocity)
            override fun feed(): Command = parallel(feed.on(), intake.on())
            override fun allOff(): Command = parallel(shooter.off(), feed.off(), intake.off())
        }
        val runner = RouteCommands(route, drive, mechanisms)

        val startPose = route.start.toPedro()
        follower.setPose(startPose)
        showRoute(route)

        waitForStart()
        follower.setPose(startPose)
        runner.command().schedule()

        try {
            while (opModeIsActive()) {
                follower.update()
                Scheduler.execute()

                val current = follower.pose()
                telemetry.addData("Step", runner.status)
                telemetry.addData("Flywheel (ticks/s)", "%.0f", shooter.velocity)
                if (runner.spinUpTimedOut) telemetry.addLine("Spin-up timed out; fed below target.")
                telemetry.addData("X (in)", "%.1f", current.x())
                telemetry.addData("Y (in)", "%.1f", current.y())
                telemetry.addData("Heading (deg)", "%.1f", Math.toDegrees(current.heading()))
                telemetry.update()
            }
        } finally {
            Scheduler.reset()
            follower.stop()
            mechanisms.allOff().schedule()
            Scheduler.execute()
        }
    }

    private fun RoutePose.toPedro() = Pose(x, y, heading)

    private fun toPath(leg: Leg): Path {
        val points = leg.controls.map { Vector2D.cartesian(it.x, it.y) }
        val path = if (points.size == 2) line(points[0], points[1]) else curve(*points.toTypedArray())
        return when (val h = leg.heading) {
            is LegHeading.Constant -> path.constant(h.radians)
            is LegHeading.Linear -> path.linear(h.from, h.to)
            LegHeading.Tangent -> path.tangent()
            LegHeading.ReverseTangent -> path.reverseTangent()
        }
    }

    private fun showRoute(route: Route) {
        if (route.warnings.isNotEmpty()) {
            telemetry.addLine("!! CHECK ROUTE !!")
            route.warnings.forEach { telemetry.addLine("!! $it") }
            telemetry.addLine()
        }
        telemetry.addLine("Start: ${route.startName}")
        telemetry.addLine("  (%.0f, %.0f) %.0f deg".format(
            route.start.x, route.start.y, Math.toDegrees(route.start.heading)))
        route.steps.forEach { s ->
            if (s is Leg) {
                val end = s.controls.last()
                telemetry.addLine("${s.label}  (%.0f, %.0f)".format(end.x, end.y))
            } else {
                telemetry.addLine(s.label)
            }
        }
        telemetry.update()
    }

    private fun showError(message: String) {
        telemetry.addLine("ROUTE ERROR, robot will not move:")
        telemetry.addLine(message)
        telemetry.update()
        waitForStart()
        while (opModeIsActive()) {
            telemetry.addLine("ROUTE ERROR, robot will not move:")
            telemetry.addLine(message)
            telemetry.update()
            idle()
        }
    }
}
