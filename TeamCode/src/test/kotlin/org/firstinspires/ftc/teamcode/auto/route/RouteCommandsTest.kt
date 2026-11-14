package org.firstinspires.ftc.teamcode.auto.route

import com.pedropathing.ivy.Command
import com.pedropathing.ivy.Scheduler
import com.pedropathing.ivy.commands.Commands.instant
import com.pedropathing.ivy.groups.Groups.parallel
import org.junit.After
import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertTrue
import org.junit.Before
import org.junit.Test

/** Runs routes through the real Ivy scheduler against fake hardware and records what happens. */
class RouteCommandsTest {
    private val events = mutableListOf<String>()

    /** Each leg finishes after [loopsPerLeg] scheduler loops. */
    private inner class FakeDrive(private val loopsPerLeg: Int = 3) : RouteDrive {
        private var loopsLeft = 0
        override fun follow(leg: Leg) {
            events += "follow ${leg.label}"
            loopsLeft = loopsPerLeg
        }
        override fun atLegEnd() = --loopsLeft <= 0
        override fun hold(pose: RoutePose) {
            events += "hold"
        }
        override fun stop() {
            events += "stop"
        }
    }

    // Same shape as the real Shooter, Feed, and Intake commands: instant, each requiring its subsystem.
    private val shooter = Any()
    private val feeder = Any()
    private val intake = Any()

    /** The flywheel gains [rampPerLoop] ticks/s each loop toward its target. */
    private inner class FakeMechanisms(private val rampPerLoop: Double = 400.0) : RouteMechanisms {
        private var target = 0.0
        override var flywheelVelocity = 0.0
        fun tick() {
            flywheelVelocity = minOf(target, flywheelVelocity + rampPerLoop)
        }
        override fun spinUp(velocity: Double): Command = instant {
            events += "spinUp %.0f".format(velocity)
            target = velocity
        }.requiring(shooter)
        override fun feed(): Command = parallel(
            instant { events += "feed at %.0f".format(flywheelVelocity) }.requiring(feeder),
            instant {}.requiring(intake),
        )
        override fun allOff(): Command = parallel(
            instant {
                events += "allOff"
                target = 0.0
                flywheelVelocity = 0.0
            }.requiring(shooter),
            instant {}.requiring(feeder),
            instant {}.requiring(intake),
        )
    }

    private fun run(runner: RouteCommands, mechanisms: FakeMechanisms) {
        runner.command().schedule()
        val deadline = System.currentTimeMillis() + 5000
        while (!runner.finished && System.currentTimeMillis() < deadline) {
            Scheduler.execute()
            mechanisms.tick()
            Thread.sleep(1)
        }
        assertTrue("route did not finish; events: $events", runner.finished)
    }

    @Before fun reset() = Scheduler.reset()

    @After fun cleanUp() = Scheduler.reset()

    @Test fun runsStepsInOrder() {
        val route = RoutePlanner.plan(route(start = C6 against FAR, facing = FAR) {
            spinUp(velocity = 1137)
            driveTo(C5, facing = RED)
            wait(20)
            shoot(feedSeconds = 0.02)
            driveTo(at(A, 5.5).offWall(RED, gap = 6))
        })
        val mechanisms = FakeMechanisms()
        val runner = RouteCommands(route, FakeDrive(), mechanisms, holdMs = 10.0)
        run(runner, mechanisms)

        assertEquals(
            listOf(
                "spinUp 1137",
                "follow Leg 1: C5 facing RED",
                "stop",
                "stop",
                "spinUp 1137",
                "feed at 1137",
                "allOff",
                "follow Leg 2: at(A, 5.5) 6 in off RED",
                "hold",
                "stop",
            ),
            events,
        )
        assertFalse(runner.spinUpTimedOut)
        assertEquals("Done (stopped)", runner.status)
    }

    @Test fun waitsForSpeedBeforeFeeding() {
        val route = RoutePlanner.plan(route(start = C1, facing = FAR) { shoot(velocity = 1137, feedSeconds = 0.02) })
        val mechanisms = FakeMechanisms(rampPerLoop = 50.0)
        run(RouteCommands(route, FakeDrive(), mechanisms, holdMs = 10.0), mechanisms)
        assertTrue(events.toString(), events.any { it.startsWith("feed at") && it.removePrefix("feed at ").toInt() >= 1112 })
    }

    @Test fun feedsAnywayAfterTimeout() {
        val route = RoutePlanner.plan(route(start = C1, facing = FAR) { shoot(velocity = 1137, feedSeconds = 0.02) })
        val mechanisms = FakeMechanisms(rampPerLoop = 0.0)
        val runner = RouteCommands(route, FakeDrive(), mechanisms, spinUpTimeoutMs = 30.0, holdMs = 10.0)
        run(runner, mechanisms)
        assertTrue(events.contains("feed at 0"))
        assertTrue(runner.spinUpTimedOut)
    }
}
