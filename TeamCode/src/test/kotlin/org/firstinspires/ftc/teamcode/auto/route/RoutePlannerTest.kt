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

    @Test fun tileCenter() = assertPoint(60.0, 12.0, C1)
    @Test fun farTile() = assertPoint(132.0, 132.0, F6)
    @Test fun corner() = assertPoint(24.0, 48.0, A_B / R2_3)
    @Test fun columnEdge() = assertPoint(24.0, 36.0, A_B / R2)
    @Test fun rowEdge() = assertPoint(60.0, 48.0, C / R2_3)
    @Test fun inchesPoint() = assertPoint(30.0, 36.5, inches(30, 36.5))
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
        assertEquals("F2 6 in off BLUE", F2.offWall(BLUE, gap = 6).name)
    }

    @Test fun names() {
        assertEquals("C1", C1.name)
        assertEquals("A_B / R5", (A_B / R5).name)
        assertEquals("C1 against AUDIENCE", (C1 against AUDIENCE).name)
    }

    @Test fun redPark() {
        val route = RoutePlanner.plan(route(start = C1 against AUDIENCE, facing = AUDIENCE) {
            driveTo(A_B / R1_2)
            driveTo(A_B / R5, facing = BLUE)
            driveTo(A5)
        })
        assertEquals(RoutePose(60.0, 9.0, -PI / 2), route.start)
        assertEquals(LegHeading.Constant(-PI / 2), route.legs[0].heading)
        assertEquals(LegHeading.Linear(-PI / 2, 0.0), route.legs[1].heading)
        assertEquals("Leg 2: A_B / R5 facing BLUE", route.legs[1].label)
        assertEquals(LegHeading.Constant(0.0), route.legs[2].heading)
        assertEquals(RoutePose(12.0, 108.0, 0.0), route.end)
        assertEquals(emptyList<String>(), route.warnings)
    }

    @Test fun bluePark() {
        val route = RoutePlanner.plan(route(start = D6 against FAR, facing = FAR) {
            driveTo(E_F / R5_6)
            driveTo(E_F / R2, facing = RED)
            driveTo(F2)
        })
        assertEquals(RoutePose(84.0, 135.0, PI / 2), route.start)
        assertEquals(LegHeading.Linear(PI / 2, PI), route.legs[1].heading)
        assertEquals(132.0, route.end.x, 1e-9)
        assertEquals(36.0, route.end.y, 1e-9)
        assertEquals(PI, route.end.heading, 1e-9)
        assertEquals(emptyList<String>(), route.warnings)
    }

    @Test fun turnTakesShortWay() {
        // From facing red (180) to facing audience (-90) is a 90 degree turn, not 270.
        val route = RoutePlanner.plan(route(start = A1, facing = RED) { driveTo(A2, facing = AUDIENCE) })
        assertEquals(LegHeading.Linear(PI, 1.5 * PI), route.legs[0].heading)
    }

    @Test fun shootThenDrive() {
        val route = RoutePlanner.plan(route(start = D6 against FAR, facing = FAR) {
            shoot(velocity = 1137, feedSeconds = 3)
            driveTo(E_F / R5_6)
        })
        assertEquals(Shoot("Shoot: 1137 ticks/s, feed 3.0 s", 1137.0, 3.0), route.steps[0])
        assertEquals("Leg 1: E_F / R5_6", route.steps[1].label)
        assertEquals(emptyList<String>(), route.warnings)
    }

    @Test fun shootOnlyEndsAtStart() {
        val route = RoutePlanner.plan(route(start = C1, facing = FAR) { shoot(velocity = 1137, feedSeconds = 3) })
        assertEquals(route.start, route.end)
    }

    @Test fun warnsFastShot() {
        val route = RoutePlanner.plan(route(start = C1, facing = FAR) { shoot(velocity = 1600, feedSeconds = 3) })
        assertTrue(route.warnings.single().contains("drained battery"))
    }

    @Test fun badShots() {
        assertError("velocity") { RoutePlanner.plan(route(start = C1, facing = FAR) { shoot(0, 3) }) }
        assertError("feed time") { RoutePlanner.plan(route(start = C1, facing = FAR) { shoot(1137, 0) }) }
    }

    @Test fun waitsAnywhere() {
        val route = RoutePlanner.plan(route(start = D6 against FAR, facing = FAR) {
            wait(250)
            shoot(velocity = 1137, feedSeconds = 3)
            wait(500)
            driveTo(E_F / R5_6)
            wait(100)
            driveTo(E_F / R2, facing = RED)
        })
        assertEquals(Wait("Wait 250 ms", 250), route.steps[0])
        assertEquals(Wait("Wait 500 ms", 500), route.steps[2])
        assertEquals("Leg 2: E_F / R2 facing RED", route.steps[5].label)
        assertEquals(LegHeading.Linear(PI / 2, PI), route.legs[1].heading)
    }

    @Test fun farStartParkRed() {
        val route = RoutePlanner.plan(route(start = C6 against FAR, facing = AUDIENCE) {
            wait(8000)
            shoot(velocity = 1137, feedSeconds = 3)
            driveTo(A5.offWall(RED, gap = 6))
        })
        assertEquals(RoutePose(60.0, 135.0, -PI / 2), route.start)
        assertEquals(RoutePose(15.0, 108.0, -PI / 2), route.end)
        assertEquals(emptyList<String>(), route.warnings)
    }

    @Test fun badWait() {
        assertError("above 0 ms") { RoutePlanner.plan(route(start = C1, facing = FAR) { wait(0) }) }
    }

    @Test fun degreesHeading() {
        val route = RoutePlanner.plan(route(start = A1, facing = degrees(45)) { driveTo(A2) })
        assertEquals(PI / 4, route.start.heading, 1e-9)
    }

    @Test fun curveKeepsViaPoints() {
        val route = RoutePlanner.plan(route(start = B1, facing = FAR) {
            curveTo(A_B / R5, via = inches(24, 60), facing = FORWARD)
        })
        assertEquals(3, route.legs[0].controls.size)
        assertEquals(LegHeading.Tangent, route.legs[0].heading)
        assertEquals(PI / 2, route.end.heading, 1e-9)
    }

    @Test fun warnsThroughHive() {
        val route = RoutePlanner.plan(route(start = C1, facing = FAR) { driveTo(C6) })
        assertTrue(route.warnings.single().contains("through the HIVE"))
    }

    @Test fun warnsCloseToHive() {
        val route = RoutePlanner.plan(route(start = B1, facing = FAR) { driveTo(B6) })
        assertTrue(route.warnings.single().contains("from the HIVE"))
    }

    @Test fun warnsOffWall() {
        val route = RoutePlanner.plan(route(start = A1, facing = FAR) { driveTo(inches(4, 12)) })
        assertTrue(route.warnings.single().contains("from a wall"))
    }

    @Test fun routeErrors() {
        assertError("at least one step") { RoutePlanner.plan(route(start = C1, facing = FAR) {}) }
        assertError("already is") { RoutePlanner.plan(route(start = C1, facing = FAR) { driveTo(C1) }) }
    }
}
