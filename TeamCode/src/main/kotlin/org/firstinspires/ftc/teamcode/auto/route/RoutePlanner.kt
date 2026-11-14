package org.firstinspires.ftc.teamcode.auto.route

import kotlin.math.PI
import kotlin.math.abs
import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.hypot
import kotlin.math.max
import kotlin.math.sin

/**
 * Turns a [RouteSpec] into drive legs and checks it against the HIVE and the walls. Plain Kotlin
 * with no SDK or Pedro types, so it can be unit tested.
 *
 * A leg without a facing keeps the previous heading. A new heading turns smoothly over the leg,
 * the short way round.
 */
object RoutePlanner {
    const val FIELD = 144.0

    /** Half the robot's length and width. */
    const val ROBOT_HALF = 9.0

    /** The HIVE fills tiles C3:D4. */
    const val HIVE_MIN = 48.0
    const val HIVE_MAX = 96.0

    /** Warn when the robot passes closer than this to the HIVE, inches. */
    const val HIVE_MARGIN = 2.0

    /** Warn when the route ends closer than this to a wall, inches. Touching is a penalty. */
    const val END_WALL_GAP = 0.5

    private const val SAMPLES = 60

    /** Flywheel top speed on a drained battery, ticks/s. */
    const val SAFE_FLYWHEEL_SPEED = 1460.0

    fun plan(spec: RouteSpec): Route {
        if (spec.steps.isEmpty()) throw RouteException("The route needs at least one step after the start.")

        var at = spec.start
        var heading = spec.facing.radians
        val warnings = mutableListOf<String>()
        warnings += clearanceWarnings("Start", listOf(RoutePose(at.x, at.y, heading)))

        var legCount = 0
        var spinning: Double? = null
        fun checkVelocity(label: String, velocity: Double) {
            if (velocity <= 0.0) throw RouteException("$label: velocity must be above 0.")
            if (velocity > SAFE_FLYWHEEL_SPEED) {
                warnings += "$label: a drained battery tops out near %.0f ticks/s.".format(SAFE_FLYWHEEL_SPEED)
            }
        }

        val steps = spec.steps.map<StepSpec, RouteStep> { item ->
            val step = when (item) {
                is StepSpec.SpinUp -> {
                    val label = "Spin up: %.0f ticks/s".format(item.velocity)
                    checkVelocity(label, item.velocity)
                    spinning = item.velocity
                    return@map SpinUp(label, item.velocity)
                }
                is StepSpec.Shoot -> {
                    val velocity = item.velocity ?: spinning ?: throw RouteException(
                        "shoot(feedSeconds = ...) needs a spinUp(...) before it, or give it a velocity."
                    )
                    val label = "Shoot: %.0f ticks/s, feed %.1f s".format(velocity, item.feedSeconds)
                    checkVelocity(label, velocity)
                    if (item.feedSeconds <= 0.0) throw RouteException("$label: feed time must be above 0.")
                    spinning = null
                    return@map Shoot(label, velocity, item.feedSeconds)
                }
                is StepSpec.Wait -> {
                    val label = "Wait ${item.ms} ms"
                    if (item.ms <= 0) throw RouteException("$label: wait must be above 0 ms.")
                    return@map Wait(label, item.ms)
                }
                is StepSpec.Drive -> item
            }
            legCount++
            val label = "Leg $legCount: ${step.point.name}" + (step.facing?.let { " facing $it" } ?: "")
            val controls = listOf(at) + step.via + step.point
            if (step.via.isEmpty() && hypot(step.point.x - at.x, step.point.y - at.y) < 0.5) {
                throw RouteException("$label is where the robot already is.")
            }

            val last = controls[controls.size - 2]
            val pathAngle = atan2(step.point.y - last.y, step.point.x - last.x)
            val (legHeading, endHeading) = when (val f = step.facing) {
                null -> LegHeading.Constant(heading) to heading
                is FixedFacing -> {
                    val turn = wrap(f.radians - heading)
                    if (abs(turn) < 1e-6) LegHeading.Constant(heading) to heading
                    else LegHeading.Linear(heading, heading + turn) to heading + turn
                }
                FORWARD -> LegHeading.Tangent to pathAngle
                BACKWARD -> LegHeading.ReverseTangent to pathAngle + PI
            }

            val poses = (0..SAMPLES).map { i ->
                val t = i.toDouble() / SAMPLES
                val p = bezier(controls, t)
                RoutePose(p.x, p.y, headingAt(legHeading, controls, t))
            }
            warnings += clearanceWarnings(label, poses)

            at = step.point
            heading = endHeading
            Leg(label, controls, legHeading)
        }

        if (spinning != null) {
            warnings += "The flywheels are still spinning at the end. Add a shoot(...) after spinUp(...)."
        }

        val end = RoutePose(at.x, at.y, heading)
        if (wallGap(end) < END_WALL_GAP) {
            warnings += "The route ends touching a wall, which is a penalty. Use offWall(...)."
        }

        return Route(
            start = RoutePose(spec.start.x, spec.start.y, spec.facing.radians),
            startName = "${spec.start.name} facing ${spec.facing}",
            steps = steps,
            end = end,
            warnings = warnings.distinct(),
        )
    }

    /**
     * How far the robot's footprint sticks out from its center along x and y. 9 in when square to
     * the field, up to 12.7 in at 45 degrees.
     */
    private fun reach(heading: Double) = ROBOT_HALF * (abs(cos(heading)) + abs(sin(heading)))

    /** Gap from the robot's footprint to the nearest wall. Negative means into the wall. */
    private fun wallGap(p: RoutePose): Double {
        val r = reach(p.heading)
        return minOf(p.x - r, p.y - r, FIELD - p.x - r, FIELD - p.y - r)
    }

    /** Gap from the robot's footprint to the HIVE. 0 means touching or overlapping. */
    private fun hiveGap(p: RoutePose): Double {
        val r = reach(p.heading)
        val dx = max(0.0, max(HIVE_MIN - (p.x + r), (p.x - r) - HIVE_MAX))
        val dy = max(0.0, max(HIVE_MIN - (p.y + r), (p.y - r) - HIVE_MAX))
        return hypot(dx, dy)
    }

    private fun clearanceWarnings(label: String, poses: List<RoutePose>): List<String> {
        val warnings = mutableListOf<String>()
        val hive = poses.minOf(::hiveGap)
        if (hive <= 0.0) warnings += "$label hits the HIVE."
        else if (hive < HIVE_MARGIN) warnings += "$label passes %.1f in from the HIVE; keep at least %.0f."
            .format(hive, HIVE_MARGIN)
        val wall = poses.minOf(::wallGap)
        if (wall < -0.01) warnings += "$label goes %.1f in into a wall.".format(-wall)
        return warnings
    }

    private fun headingAt(heading: LegHeading, controls: List<FieldPoint>, t: Double): Double = when (heading) {
        is LegHeading.Constant -> heading.radians
        is LegHeading.Linear -> heading.from + (heading.to - heading.from) * t
        LegHeading.Tangent, LegHeading.ReverseTangent -> {
            // A Bezier curve's direction is the Bezier of its control point differences.
            val d = bezier(controls.zipWithNext { a, b -> FieldPoint(b.x - a.x, b.y - a.y, "") }, t)
            atan2(d.y, d.x) + if (heading == LegHeading.ReverseTangent) PI else 0.0
        }
    }

    private fun bezier(controls: List<FieldPoint>, t: Double): FieldPoint {
        var pts = controls
        while (pts.size > 1) {
            pts = pts.zipWithNext { a, b -> FieldPoint(a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t, "") }
        }
        return pts[0]
    }

    private fun wrap(angle: Double): Double {
        var a = angle
        while (a > PI) a -= 2 * PI
        while (a <= -PI) a += 2 * PI
        return a
    }
}

class RouteException(message: String) : IllegalArgumentException(message)

data class RoutePose(val x: Double, val y: Double, val heading: Double)

/** One thing the auto does, in order. */
sealed class RouteStep {
    abstract val label: String
}

/** One leg to follow. [controls] holds the start, any via points, then the end. */
data class Leg(
    override val label: String,
    val controls: List<FieldPoint>,
    val heading: LegHeading,
) : RouteStep()

/** Start the flywheels toward [velocity] ticks/s without waiting. */
data class SpinUp(override val label: String, val velocity: Double) : RouteStep()

/** Spin the flywheels to [velocity] ticks/s, then feed for [feedSeconds]. */
data class Shoot(override val label: String, val velocity: Double, val feedSeconds: Double) : RouteStep()

/** Stop and brake for [ms] milliseconds. */
data class Wait(override val label: String, val ms: Int) : RouteStep()

data class Route(
    val start: RoutePose,
    val startName: String,
    val steps: List<RouteStep>,
    val end: RoutePose,
    val warnings: List<String>,
) {
    val legs: List<Leg> get() = steps.filterIsInstance<Leg>()
}

sealed class LegHeading {
    data class Constant(val radians: Double) : LegHeading()
    data class Linear(val from: Double, val to: Double) : LegHeading()
    object Tangent : LegHeading()
    object ReverseTangent : LegHeading()
}
