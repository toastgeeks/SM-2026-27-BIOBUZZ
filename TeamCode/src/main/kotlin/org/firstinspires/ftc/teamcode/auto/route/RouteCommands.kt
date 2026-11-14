package org.firstinspires.ftc.teamcode.auto.route

import com.pedropathing.ivy.Command
import com.pedropathing.ivy.commands.Commands.instant
import com.pedropathing.ivy.commands.Commands.waitMs
import com.pedropathing.ivy.commands.Commands.waitUntil
import com.pedropathing.ivy.groups.Groups.sequential

/** What a route needs from the drivetrain. */
interface RouteDrive {
    /** Start following [leg]. */
    fun follow(leg: Leg)

    /** True once the current leg is finished. */
    fun atLegEnd(): Boolean

    fun hold(pose: RoutePose)

    /** Stop and brake. */
    fun stop()
}

/** What a route needs from the shooter, feed, and intake. */
interface RouteMechanisms {
    /** Flywheel speed in ticks/s. */
    val flywheelVelocity: Double

    /** Set the flywheels' target speed. */
    fun spinUp(velocity: Double): Command

    /** Run the feed and the intake. */
    fun feed(): Command

    /** Stop the flywheels, feed, and intake. */
    fun allOff(): Command
}

/**
 * Turns a planned [Route] into one Ivy command: each step in order, then hold the end pose for a
 * moment and stop. Holding forever would push into a wall when the end pose is a little off the
 * real field.
 *
 * No SDK or Pedro types, so it runs in unit tests with fake hardware.
 */
class RouteCommands(
    private val route: Route,
    private val drive: RouteDrive,
    private val mechanisms: RouteMechanisms,
    private val spinUpTimeoutMs: Double = SPIN_UP_TIMEOUT_MS,
    private val holdMs: Double = HOLD_MS,
) {
    /** What the robot is doing now, for telemetry. */
    var status = "Not started"
        private set

    /** True if a shot fed before the flywheels reached speed. */
    var spinUpTimedOut = false
        private set

    /** True once the route is done and the drivetrain has stopped. */
    var finished = false
        private set

    fun command(): Command = sequential(*(route.steps.map(::step) + finish()).toTypedArray())

    private fun step(step: RouteStep): Command = sequential(
        instant { status = step.label },
        when (step) {
            is Leg -> Command.build()
                .setStart { drive.follow(step) }
                .setExecute {}
                .setDone { drive.atLegEnd() }
                .setEnd {}
            is Wait -> sequential(instant { drive.stop() }, waitMs(step.ms.toDouble()))
            is SpinUp -> mechanisms.spinUp(step.velocity)
            is Shoot -> shoot(step)
        },
    )

    private fun shoot(shot: Shoot): Command {
        val ready = { mechanisms.flywheelVelocity >= shot.velocity - SPIN_UP_TOLERANCE }
        return sequential(
            // Braking, not holding, so the robot can't push into a wall while it shoots.
            instant {
                drive.stop()
                status = "${shot.label} (spinning up)"
            },
            mechanisms.spinUp(shot.velocity),
            waitUntil { ready() }.raceWith(waitMs(spinUpTimeoutMs)),
            instant {
                if (!ready()) spinUpTimedOut = true
                status = "${shot.label} (feeding)"
            },
            mechanisms.feed(),
            waitMs(shot.feedSeconds * 1000.0),
            mechanisms.allOff(),
        )
    }

    private fun finish(): Command = sequential(
        instant {
            status = "Done (settling)"
            drive.hold(route.end)
        },
        waitMs(holdMs),
        instant {
            drive.stop()
            status = "Done (stopped)"
            finished = true
        },
    )

    companion object {
        /** Feed once the flywheels are within this many ticks/s below the target (about 2%). */
        const val SPIN_UP_TOLERANCE = 25.0

        /** Feed anyway after this long; 1.6 s reaches 1125 on a fresh battery. */
        const val SPIN_UP_TIMEOUT_MS = 4000.0

        /** How long to hold the last pose before stopping the drivetrain. */
        const val HOLD_MS = 1000.0
    }
}
