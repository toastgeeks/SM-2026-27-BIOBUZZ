@file:Suppress("unused")

package org.firstinspires.ftc.teamcode.auto.route

import kotlin.math.PI
import kotlin.math.floor

/*
 * Names for places on the field, for writing routes.
 *
 * Every spot is `at(column, row)`. Whole numbers are tile centers, and halves are the lines
 * between tiles:
 *
 *  - `at(B, 6)`: center of B6. `B6` is shorthand for the same spot.
 *  - `at(B, 5.5)`: in column B, on the line between rows 5 and 6.
 *  - `at(E + 0.5, 5.5)`: the corner where E5, E6, F5 and F6 meet.
 *  - `at(A, 5.25)`: anywhere in between.
 *  - `C6 against FAR`: slid so the robot touches that wall.
 *  - `A5.offWall(RED, gap = 6)`: slid so the robot stops 6 in short of that wall.
 *
 * Inches underneath: origin in the A1 corner, 24 in tiles, heading 0 faces the blue (F) side.
 */

private const val TILE = 24.0

/** A column position. A to F are tile centers; `A + 0.5` is the line between A and B. */
class Column internal constructor(val index: Double) {
    operator fun plus(tiles: Number) = Column(index + tiles.toDouble())
    operator fun minus(tiles: Number) = Column(index - tiles.toDouble())

    internal val inches: Double get() = (index - 0.5) * TILE

    override fun toString(): String {
        val base = floor(index).toInt().coerceIn(1, 6)
        val offset = index - base
        val letter = ('A' + base - 1).toString()
        return when {
            offset == 0.0 -> letter
            offset > 0.0 -> "$letter+${number(offset)}"
            else -> "$letter${number(offset)}"
        }
    }
}

val A = Column(1.0)
val B = Column(2.0)
val C = Column(3.0)
val D = Column(4.0)
val E = Column(5.0)
val F = Column(6.0)

/** A spot on the field in inches. [name] is for telemetry. */
data class FieldPoint(val x: Double, val y: Double, val name: String = "(%.0f, %.0f)".format(x, y))

/** [row] 6 is the center of row 6; 5.5 is the line between rows 5 and 6. */
fun at(column: Column, row: Number): FieldPoint {
    val r = row.toDouble()
    return FieldPoint(column.inches, (r - 0.5) * TILE, "at($column, ${number(r)})")
}

private fun number(value: Double): String = value.toString().removeSuffix(".0")

/** Slides the point so the robot touches [wall]. */
infix fun FieldPoint.against(wall: Side): FieldPoint {
    val half = RoutePlanner.ROBOT_HALF
    val moved = when (wall) {
        Side.AUDIENCE -> copy(y = half)
        Side.FAR -> copy(y = RoutePlanner.FIELD - half)
        Side.RED -> copy(x = half)
        Side.BLUE -> copy(x = RoutePlanner.FIELD - half)
    }
    return moved.copy(name = "$name against $wall")
}

/** Slides the point so the robot stops [gap] inches short of [wall]. */
fun FieldPoint.offWall(wall: Side, gap: Number): FieldPoint {
    val g = gap.toDouble()
    val touching = against(wall)
    val moved = when (wall) {
        Side.AUDIENCE -> touching.copy(y = touching.y + g)
        Side.FAR -> touching.copy(y = touching.y - g)
        Side.RED -> touching.copy(x = touching.x + g)
        Side.BLUE -> touching.copy(x = touching.x - g)
    }
    return moved.copy(name = "$name ${number(g)} in off $wall")
}

/** Which way the robot faces during a leg. */
sealed interface Facing

/** A heading that doesn't depend on the path. The start needs one of these. */
sealed interface FixedFacing : Facing {
    val radians: Double
}

/** A side of the field: a wall to touch or a way to face. */
enum class Side(override val radians: Double) : FixedFacing {
    AUDIENCE(-PI / 2), FAR(PI / 2), RED(PI), BLUE(0.0)
}

class Degrees internal constructor(val degrees: Double) : FixedFacing {
    override val radians get() = degrees * PI / 180.0
    override fun toString() = "%.0f deg".format(degrees)
}

fun degrees(degrees: Number): FixedFacing = Degrees(degrees.toDouble())

/** Front of the robot leads along the path. */
object FORWARD : Facing {
    override fun toString() = "FORWARD"
}

/** Back of the robot leads along the path. */
object BACKWARD : Facing {
    override fun toString() = "BACKWARD"
}

val AUDIENCE = Side.AUDIENCE
val FAR = Side.FAR
val RED = Side.RED
val BLUE = Side.BLUE

private fun tile(column: Column, row: Int) = at(column, row).copy(name = "$column$row")

val A1 = tile(A, 1); val B1 = tile(B, 1); val C1 = tile(C, 1)
val D1 = tile(D, 1); val E1 = tile(E, 1); val F1 = tile(F, 1)
val A2 = tile(A, 2); val B2 = tile(B, 2); val C2 = tile(C, 2)
val D2 = tile(D, 2); val E2 = tile(E, 2); val F2 = tile(F, 2)
val A3 = tile(A, 3); val B3 = tile(B, 3); val C3 = tile(C, 3)
val D3 = tile(D, 3); val E3 = tile(E, 3); val F3 = tile(F, 3)
val A4 = tile(A, 4); val B4 = tile(B, 4); val C4 = tile(C, 4)
val D4 = tile(D, 4); val E4 = tile(E, 4); val F4 = tile(F, 4)
val A5 = tile(A, 5); val B5 = tile(B, 5); val C5 = tile(C, 5)
val D5 = tile(D, 5); val E5 = tile(E, 5); val F5 = tile(F, 5)
val A6 = tile(A, 6); val B6 = tile(B, 6); val C6 = tile(C, 6)
val D6 = tile(D, 6); val E6 = tile(E, 6); val F6 = tile(F, 6)

/** What a route says, before [RoutePlanner] checks it. Build one with [route]. */
class RouteSpec internal constructor(
    val start: FieldPoint,
    val facing: FixedFacing,
    internal val steps: List<StepSpec>,
)

internal sealed class StepSpec {
    class Drive(val point: FieldPoint, val facing: Facing?, val via: List<FieldPoint>) : StepSpec()
    class SpinUp(val velocity: Double) : StepSpec()
    class Shoot(val velocity: Double?, val feedSeconds: Double) : StepSpec()
    class Wait(val ms: Int) : StepSpec()
}

class RouteBuilder internal constructor() {
    internal val steps = mutableListOf<StepSpec>()

    /** Straight line to [point]. Without [facing], the robot keeps its heading. */
    fun driveTo(point: FieldPoint, facing: Facing? = null) {
        steps += StepSpec.Drive(point, facing, emptyList())
    }

    /**
     * Curve to [point], bent toward [via] (and [more]). The robot does not pass through the via
     * points; they pull the curve toward them.
     */
    fun curveTo(point: FieldPoint, via: FieldPoint, vararg more: FieldPoint, facing: Facing? = null) {
        steps += StepSpec.Drive(point, facing, listOf(via) + more)
    }

    /**
     * Start the flywheels toward [velocity] (ticks/s) and go straight on to the next step, so they
     * spin up while the robot drives. A later `shoot(feedSeconds = ...)` uses this velocity.
     */
    fun spinUp(velocity: Number) {
        steps += StepSpec.SpinUp(velocity.toDouble())
    }

    /**
     * Stop where the robot is, wait for the flywheels to reach [velocity] (ticks/s), run the feed
     * and intake for [feedSeconds], then stop the flywheels, feed, and intake.
     */
    fun shoot(velocity: Number, feedSeconds: Number) {
        steps += StepSpec.Shoot(velocity.toDouble(), feedSeconds.toDouble())
    }

    /** Like `shoot(velocity, feedSeconds)`, at the velocity from the last [spinUp]. */
    fun shoot(feedSeconds: Number) {
        steps += StepSpec.Shoot(null, feedSeconds.toDouble())
    }

    /** Stop and brake where the robot is for [ms] milliseconds. */
    fun wait(ms: Int) {
        steps += StepSpec.Wait(ms)
    }
}

fun route(start: FieldPoint, facing: FixedFacing, legs: RouteBuilder.() -> Unit): RouteSpec =
    RouteSpec(start, facing, RouteBuilder().apply(legs).steps.toList())
