package org.firstinspires.ftc.teamcode.auto

import com.pedropathing.api.Paths.curve
import com.pedropathing.api.Paths.line
import com.pedropathing.ivy.Scheduler
import com.pedropathing.math.Pose
import com.pedropathing.math.Vector2D
import com.pedropathing.paths.Path
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.auto.route.Leg
import org.firstinspires.ftc.teamcode.auto.route.LegHeading
import org.firstinspires.ftc.teamcode.auto.route.Route
import org.firstinspires.ftc.teamcode.auto.route.RouteException
import org.firstinspires.ftc.teamcode.auto.route.RoutePlanner
import org.firstinspires.ftc.teamcode.auto.route.RouteSpec
import org.firstinspires.ftc.teamcode.auto.route.Shoot
import org.firstinspires.ftc.teamcode.auto.route.Wait
import org.firstinspires.ftc.teamcode.pedro.Constants
import org.firstinspires.ftc.teamcode.subsystems.Feed
import org.firstinspires.ftc.teamcode.subsystems.Intake
import org.firstinspires.ftc.teamcode.subsystems.Shooter

/**
 * Auto that runs a route built with `route { ... }` (see Field.kt): drive legs and shots, in
 * order. Routes are in field coordinates, so red and blue each write their own.
 *
 * INIT shows the route and any HIVE, wall, or flywheel warnings on the Driver Station. A route
 * that can't be read shows the error and the robot does not move.
 */
abstract class RouteAuto(private val spec: RouteSpec) : LinearOpMode() {

    private enum class ShotPhase { SPIN_UP, FEED }

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

        val startPose = Pose(route.start.x, route.start.y, route.start.heading)
        val endPose = Pose(route.end.x, route.end.y, route.end.heading)
        val paths = route.steps.map { (it as? Leg)?.let(::toPath) }

        follower.setPose(startPose)
        showRoute(route)

        waitForStart()
        follower.setPose(startPose)

        var step = 0
        var shotPhase = ShotPhase.SPIN_UP
        var spinUpTimedOut = false
        var stopped = false
        val timer = ElapsedTime()

        fun begin(index: Int) {
            step = index
            timer.reset()
            when (val s = route.steps.getOrNull(index)) {
                is Leg -> follower.follow(paths[index]!!)
                is Shoot -> {
                    // Idle brakes the drivetrain. Holding a pose could push into a wall while we shoot.
                    follower.stop()
                    shooter.runVelocity(s.velocity).schedule()
                    shotPhase = ShotPhase.SPIN_UP
                }
                is Wait -> follower.stop()
                // Holding forever pushes into a wall when the end pose is a little off the real
                // field, so settle briefly, then stop.
                null -> follower.hold(endPose)
            }
        }

        begin(0)

        try {
            while (opModeIsActive()) {
                follower.update()

                when (val s = route.steps.getOrNull(step)) {
                    is Leg -> if (follower.atParametricEnd()) begin(step + 1)
                    is Shoot -> when (shotPhase) {
                        ShotPhase.SPIN_UP -> {
                            val ready = shooter.velocity >= s.velocity - SPIN_UP_TOLERANCE
                            if (ready || timer.seconds() > SPIN_UP_TIMEOUT) {
                                spinUpTimedOut = !ready
                                feed.on().schedule()
                                intake.on().schedule()
                                shotPhase = ShotPhase.FEED
                                timer.reset()
                            }
                        }
                        ShotPhase.FEED -> if (timer.seconds() >= s.feedSeconds) {
                            shooter.off().schedule()
                            feed.off().schedule()
                            intake.off().schedule()
                            begin(step + 1)
                        }
                    }
                    is Wait -> if (timer.milliseconds() >= s.ms) begin(step + 1)
                    null -> if (!stopped && timer.seconds() > HOLD_SECONDS) {
                        follower.stop()
                        stopped = true
                    }
                }

                Scheduler.execute()

                val current = follower.pose()
                val s = route.steps.getOrNull(step)
                telemetry.addData("Step", when {
                    s is Shoot -> "${s.label} (${if (shotPhase == ShotPhase.SPIN_UP) "spinning up" else "feeding"})"
                    s != null -> s.label
                    stopped -> "Done (stopped)"
                    else -> "Done (settling)"
                })
                telemetry.addData("Flywheel (ticks/s)", "%.0f", shooter.velocity)
                if (spinUpTimedOut) telemetry.addLine("Spin-up timed out; fed below target.")
                telemetry.addData("X (in)", "%.1f", current.x())
                telemetry.addData("Y (in)", "%.1f", current.y())
                telemetry.addData("Heading (deg)", "%.1f", Math.toDegrees(current.heading()))
                telemetry.update()
            }
        } finally {
            follower.stop()
            shooter.off().schedule()
            feed.off().schedule()
            intake.off().schedule()
            Scheduler.execute()
        }
    }

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

    private companion object {
        /** How long to hold the last pose before stopping the drivetrain. */
        const val HOLD_SECONDS = 1.0

        /** Feed once the flywheel is within this many ticks/s below the target (about 2%). */
        const val SPIN_UP_TOLERANCE = 25.0

        /** Feed anyway after this long; 1.6 s reaches 1125 on a fresh battery. */
        const val SPIN_UP_TIMEOUT = 4.0
    }
}
