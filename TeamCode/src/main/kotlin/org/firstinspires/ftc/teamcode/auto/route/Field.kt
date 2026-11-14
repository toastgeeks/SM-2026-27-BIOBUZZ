@file:Suppress("unused", "ObjectPropertyName")

package org.firstinspires.ftc.teamcode.auto.route

import kotlin.math.PI

/*
 * Typed names for places on the field, for writing routes.
 *
 * Field frame: origin in the A1 corner, x along columns A to F, y along rows 1 to 6, inches,
 * 24 in tiles. Heading 0 faces the blue (F) side, 90 degrees faces the far wall.
 *
 *  - `C1`: tile center.
 *  - `A_B / R2_3`: corner shared by tiles A2, A3, B2 and B3.
 *  - `A_B / R2`: middle of the line between A2 and B2. `C / R2_3`: between C2 and C3.
 *  - `inches(30, 36)`: anywhere.
 *  - `C1 against AUDIENCE`: slid so the robot touches that wall.
 */

private const val TILE = 24.0

/** A line across the field at one x: a column's center line or the line between two columns. */
class Column internal constructor(val inches: Double, val name: String)

/** A line across the field at one y: a row's center line or the line between two rows. */
class Row internal constructor(val inches: Double, val name: String)

/** A spot on the field in inches. [name] is for telemetry. */
data class FieldPoint(val x: Double, val y: Double, val name: String = "(%.0f, %.0f)".format(x, y))

operator fun Column.div(row: Row) = FieldPoint(inches, row.inches, "$name / ${row.name}")

fun inches(x: Number, y: Number) = FieldPoint(x.toDouble(), y.toDouble())

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
    return moved.copy(name = "$name %.0f in off $wall".format(g))
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

private fun column(index: Int, name: String) = Column((index + 0.5) * TILE, name)
private fun columnLine(index: Int, name: String) = Column(index * TILE, name)
private fun row(index: Int, name: String) = Row((index + 0.5) * TILE, name)
private fun rowLine(index: Int, name: String) = Row(index * TILE, name)

val A = column(0, "A")
val B = column(1, "B")
val C = column(2, "C")
val D = column(3, "D")
val E = column(4, "E")
val F = column(5, "F")

val A_B = columnLine(1, "A_B")
val B_C = columnLine(2, "B_C")
val C_D = columnLine(3, "C_D")
val D_E = columnLine(4, "D_E")
val E_F = columnLine(5, "E_F")

val R1 = row(0, "R1")
val R2 = row(1, "R2")
val R3 = row(2, "R3")
val R4 = row(3, "R4")
val R5 = row(4, "R5")
val R6 = row(5, "R6")

val R1_2 = rowLine(1, "R1_2")
val R2_3 = rowLine(2, "R2_3")
val R3_4 = rowLine(3, "R3_4")
val R4_5 = rowLine(4, "R4_5")
val R5_6 = rowLine(5, "R5_6")

private fun tile(column: Column, row: Row) =
    FieldPoint(column.inches, row.inches, column.name + row.name.drop(1))

val A1 = tile(A, R1); val B1 = tile(B, R1); val C1 = tile(C, R1)
val D1 = tile(D, R1); val E1 = tile(E, R1); val F1 = tile(F, R1)
val A2 = tile(A, R2); val B2 = tile(B, R2); val C2 = tile(C, R2)
val D2 = tile(D, R2); val E2 = tile(E, R2); val F2 = tile(F, R2)
val A3 = tile(A, R3); val B3 = tile(B, R3); val C3 = tile(C, R3)
val D3 = tile(D, R3); val E3 = tile(E, R3); val F3 = tile(F, R3)
val A4 = tile(A, R4); val B4 = tile(B, R4); val C4 = tile(C, R4)
val D4 = tile(D, R4); val E4 = tile(E, R4); val F4 = tile(F, R4)
val A5 = tile(A, R5); val B5 = tile(B, R5); val C5 = tile(C, R5)
val D5 = tile(D, R5); val E5 = tile(E, R5); val F5 = tile(F, R5)
val A6 = tile(A, R6); val B6 = tile(B, R6); val C6 = tile(C, R6)
val D6 = tile(D, R6); val E6 = tile(E, R6); val F6 = tile(F, R6)

/** What a route says, before [RoutePlanner] checks it. Build one with [route]. */
class RouteSpec internal constructor(
    val start: FieldPoint,
    val facing: FixedFacing,
    internal val steps: List<StepSpec>,
)

internal sealed class StepSpec {
    class Drive(val point: FieldPoint, val facing: Facing?, val via: List<FieldPoint>) : StepSpec()
    class Shoot(val velocity: Double, val feedSeconds: Double) : StepSpec()
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
     * Stop where the robot is, spin the flywheels up to [velocity] (ticks/s), run the feed and
     * intake for [feedSeconds], then stop the flywheels, feed, and intake.
     */
    fun shoot(velocity: Number, feedSeconds: Number) {
        steps += StepSpec.Shoot(velocity.toDouble(), feedSeconds.toDouble())
    }

    /** Stop and brake where the robot is for [ms] milliseconds. */
    fun wait(ms: Int) {
        steps += StepSpec.Wait(ms)
    }
}

fun route(start: FieldPoint, facing: FixedFacing, legs: RouteBuilder.() -> Unit): RouteSpec =
    RouteSpec(start, facing, RouteBuilder().apply(legs).steps.toList())
