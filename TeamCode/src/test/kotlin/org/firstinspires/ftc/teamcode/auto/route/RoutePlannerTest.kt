package org.firstinspires.ftc.teamcode.auto.route

import org.junit.Assert.assertEquals
import org.junit.Assert.assertTrue
import org.junit.Assert.fail
import org.junit.Test
import kotlin.math.PI

class RoutePlannerTest {
    private fun assertPoint(x: Double, y: Double, p: FieldPoint) {
        assertEquals(x, p.x, 1e-9)
        assertEquals(y, p.y, 1e-9)
    }

    private fun assertError(contains: String, block: () -> Unit) {
        try {
            block()
            fail("expected a RouteException")
        } catch (e: RouteException) {
            assertTrue("\"${e.message}\" should mention \"$contains\"", e.message!!.contains(contains))
        }
    }

    @Test fun tileCenters() {
        assertPoint(60.0, 12.0, C1)
        assertPoint(132.0, 132.0, F6)
        assertEquals(at(B, 6), B6.copy(name = at(B, 6).name))
    }

    @Test fun halves() {
        assertPoint(36.0, 120.0, at(B, 5.5))       // B, between rows 5 and 6
        assertPoint(120.0, 120.0, at(E + 0.5, 5.5)) // corner of E5, E6, F5, F6
        assertPoint(24.0, 36.0, at(A + 0.5, 2))     // between A2 and B2
        assertPoint(12.0, 114.0, at(A, 5.25))
        assertPoint(0.0, 0.0, at(A - 0.5, 0.5))     // A1 corner of the field
    }

    @Test fun againstWalls() {
        assertPoint(60.0, 9.0, C1 against AUDIENCE)
        assertPoint(84.0, 135.0, D6 against FAR)
        assertPoint(9.0, 108.0, A5 against RED)
        assertPoint(135.0, 36.0, F2 against BLUE)
    }

    @Test fun offWalls() {
        assertPoint(129.0, 36.0, F2.offWall(BLUE, gap = 6))
        assertPoint(15.0, 108.0, A5.offWall(RED, gap = 6))
        assertPoint(60.0, 13.0, C1.offWall(AUDIENCE, gap = 4))
        assertPoint(84.0, 131.0, D6.offWall(FAR, gap = 4))
    }

    @Test fun names() {
        assertEquals("C1", C1.name)
        assertEquals("at(B, 6)", at(B, 6).name)
        assertEquals("at(E+0.5, 5.5)", at(E + 0.5, 5.5).name)
        assertEquals("at(A-0.25, 1)", at(A - 0.25, 1).name)
        assertEquals("C1 against AUDIENCE", (C1 against AUDIENCE).name)
        assertEquals("F2 6 in off BLUE", F2.offWall(BLUE, gap = 6).name)
    }

    @Test fun headings() {
        val route = RoutePlanner.plan(route(start = C1 against AUDIENCE, facing = AUDIENCE) {
            driveTo(at(A + 0.5, 1.5))
            driveTo(at(A + 0.5, 5), facing = BLUE)
            driveTo(A5)
        })
        assertEquals(RoutePose(60.0, 9.0, -PI / 2), route.start)
        assertEquals(LegHeading.Constant(-PI / 2), route.legs[0].heading)
        assertEquals(LegHeading.Linear(-PI / 2, 0.0), route.legs[1].heading)
        assertEquals("Leg 2: at(A+0.5, 5) facing BLUE", route.legs[1].label)
        assertEquals(LegHeading.Constant(0.0), route.legs[2].heading)
        assertEquals(RoutePose(12.0, 108.0, 0.0), route.end)
    }

    @Test fun turnTakesShortWay() {
        // From facing red (180) to facing audience (-90) is a 90 degree turn, not 270.
        val route = RoutePlanner.plan(route(start = A1, facing = RED) { driveTo(A2, facing = AUDIENCE) })
        assertEquals(LegHeading.Linear(PI, 1.5 * PI), route.legs[0].heading)
    }

    @Test fun degreesHeading() {
        val route = RoutePlanner.plan(route(start = A1, facing = degrees(45)) { driveTo(A2) })
        assertEquals(PI / 4, route.start.heading, 1e-9)
    }

    @Test fun shootAndWait() {
        val route = RoutePlanner.plan(route(start = D6 against FAR, facing = FAR) {
            wait(250)
            shoot(velocity = 1137, feedSeconds = 3)
            driveTo(at(E + 0.5, 5.5))
        })
        assertEquals(Wait("Wait 250 ms", 250), route.steps[0])
        assertEquals(Shoot("Shoot: 1137 ticks/s, feed 3.0 s", 1137.0, 3.0), route.steps[1])
        assertEquals("Leg 1: at(E+0.5, 5.5)", route.steps[2].label)
    }

    @Test fun shootUsesSpinUpVelocity() {
        val route = RoutePlanner.plan(route(start = C6 against FAR, facing = FAR) {
            spinUp(velocity = 1137)
            driveTo(C5, facing = RED)
            shoot(feedSeconds = 3)
        })
        assertEquals(SpinUp("Spin up: 1137 ticks/s", 1137.0), route.steps[0])
        assertEquals(Shoot("Shoot: 1137 ticks/s, feed 3.0 s", 1137.0, 3.0), route.steps[2])
        assertEquals(emptyList<String>(), route.warnings)
    }

    @Test fun shootNeedsAVelocity() {
        assertError("needs a spinUp") {
            RoutePlanner.plan(route(start = C1, facing = FAR) { shoot(feedSeconds = 3) })
        }
        // A shot turns the flywheels off, so the next one needs its own spinUp.
        assertError("needs a spinUp") {
            RoutePlanner.plan(route(start = C1, facing = FAR) {
                spinUp(velocity = 1137)
                shoot(feedSeconds = 3)
                shoot(feedSeconds = 3)
            })
        }
    }

    @Test fun warnsSpinningAtEnd() {
        val route = RoutePlanner.plan(route(start = C1, facing = FAR) { spinUp(velocity = 1137) })
        assertTrue(route.warnings.single().contains("still spinning"))
    }

    @Test fun shootOnlyEndsAtStart() {
        val route = RoutePlanner.plan(route(start = C1, facing = FAR) { shoot(velocity = 1137, feedSeconds = 3) })
        assertEquals(route.start, route.end)
    }

    @Test fun curveKeepsViaPoints() {
        val route = RoutePlanner.plan(route(start = B1, facing = FAR) {
            curveTo(at(A + 0.5, 5), via = at(A + 0.5, 3), facing = FORWARD)
        })
        assertEquals(3, route.legs[0].controls.size)
        assertEquals(LegHeading.Tangent, route.legs[0].heading)
        assertEquals(PI / 2, route.end.heading, 1e-9)
    }

    @Test fun warnsFastShot() {
        val route = RoutePlanner.plan(route(start = C1, facing = FAR) { shoot(velocity = 1600, feedSeconds = 3) })
        assertTrue(route.warnings.single().contains("drained battery"))
    }

    @Test fun warnsThroughHive() {
        val route = RoutePlanner.plan(route(start = C1, facing = FAR) { driveTo(C6) })
        assertTrue(route.warnings.single().contains("hits the HIVE"))
    }

    @Test fun squareRobotFitsBesideHive() {
        // Facing straight up column B, the robot's side is 3 in from the HIVE.
        val route = RoutePlanner.plan(route(start = B1, facing = FAR) { driveTo(B6) })
        assertEquals(emptyList<String>(), route.warnings)
    }

    @Test fun turningCornersCount() {
        // The same lane, but turning 90 degrees on the way swings the corners into the HIVE.
        val route = RoutePlanner.plan(route(start = B1, facing = FAR) { driveTo(B6, facing = BLUE) })
        assertTrue(route.warnings.single().contains("hits the HIVE"))
    }

    @Test fun warnsCloseToHive() {
        val route = RoutePlanner.plan(route(start = at(B + 0.1, 1), facing = FAR) { driveTo(at(B + 0.1, 6)) })
        assertTrue(route.warnings.single().contains("0.6 in from the HIVE"))
    }

    @Test fun warnsIntoWall() {
        val route = RoutePlanner.plan(route(start = A1, facing = FAR) { driveTo(at(A - 0.25, 1)) })
        assertTrue(route.warnings.any { it.contains("3.0 in into a wall") })
    }

    @Test fun warnsEndingOnWall() {
        val route = RoutePlanner.plan(route(start = C1 against AUDIENCE, facing = AUDIENCE) {
            driveTo(A1 against RED)
        })
        assertTrue(route.warnings.single().contains("ends touching a wall"))
    }

    @Test fun startingOnWallIsFine() {
        val route = RoutePlanner.plan(route(start = C1 against AUDIENCE, facing = AUDIENCE) { driveTo(B2) })
        assertEquals(emptyList<String>(), route.warnings)
    }

    @Test fun errors() {
        assertError("at least one step") { RoutePlanner.plan(route(start = C1, facing = FAR) {}) }
        assertError("already is") { RoutePlanner.plan(route(start = C1, facing = FAR) { driveTo(C1) }) }
        assertError("velocity") { RoutePlanner.plan(route(start = C1, facing = FAR) { shoot(0, 3) }) }
        assertError("feed time") { RoutePlanner.plan(route(start = C1, facing = FAR) { shoot(1137, 0) }) }
        assertError("above 0 ms") { RoutePlanner.plan(route(start = C1, facing = FAR) { wait(0) }) }
    }
}
