package org.firstinspires.ftc.teamcode.auto.route

import kotlin.math.PI
import kotlin.math.abs
import kotlin.math.atan2
import kotlin.math.hypot
import kotlin.math.max
import kotlin.math.sqrt

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

    /** Center to corner, the most the robot sticks out at any heading. */
    val ROBOT_RADIUS = ROBOT_HALF * sqrt(2.0)

    /** The HIVE fills tiles C3:D4. */
    const val HIVE_MIN = 48.0
    const val HIVE_MAX = 96.0

    private const val SAMPLES = 60

    /** Flywheel top speed on a drained battery, ticks/s. */
    const val SAFE_FLYWHEEL_SPEED = 1460.0

    fun plan(spec: RouteSpec): Route {
        if (spec.steps.isEmpty()) throw RouteException("The route needs at least one step after the start.")

        var at = spec.start
        var heading = spec.facing.radians
        val warnings = mutableListOf<String>()
        warnings += wallWarnings("Start", listOf(at))
        if (hiveGap(at) < ROBOT_RADIUS) warnings += "Start is inside or touching the HIVE."

        var legCount = 0
        val steps = spec.steps.map<StepSpec, RouteStep> { item ->
            val step = when (item) {
                is StepSpec.Shoot -> {
                    val label = "Shoot: %.0f ticks/s, feed %.1f s".format(item.velocity, item.feedSeconds)
                    if (item.velocity <= 0.0) throw RouteException("$label: velocity must be above 0.")
                    if (item.feedSeconds <= 0.0) throw RouteException("$label: feed time must be above 0.")
                    if (item.velocity > SAFE_FLYWHEEL_SPEED) {
                        warnings += "$label: a drained battery tops out near %.0f ticks/s."
                            .format(SAFE_FLYWHEEL_SPEED)
                    }
                    return@map Shoot(label, item.velocity, item.feedSeconds)
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

            val samples = (0..SAMPLES).map { bezier(controls, it.toDouble() / SAMPLES) }
            val closest = samples.minOf { hiveGap(it) }
            if (closest < ROBOT_RADIUS) {
                warnings += if (closest <= 0.0) "$label drives through the HIVE."
                else "$label passes %.1f in from the HIVE; a turning robot needs %.1f."
                    .format(closest, ROBOT_RADIUS)
            }
            warnings += wallWarnings(label, samples)

            at = step.point
            heading = endHeading
            Leg(label, controls, legHeading)
        }

        return Route(
            start = RoutePose(spec.start.x, spec.start.y, spec.facing.radians),
            startName = "${spec.start.name} facing ${spec.facing}",
            steps = steps,
            end = RoutePose(at.x, at.y, heading),
            warnings = warnings.distinct(),
        )
    }

    private fun wallWarnings(label: String, points: List<FieldPoint>): List<String> {
        val closest = points.minOf { minOf(it.x, it.y, FIELD - it.x, FIELD - it.y) }
        return if (closest < ROBOT_HALF - 0.01) {
            listOf("$label puts the robot center %.1f in from a wall; it needs %.1f.".format(closest, ROBOT_HALF))
        } else emptyList()
    }

    /** Distance from a point to the HIVE's square, 0 inside it. */
    fun hiveGap(p: FieldPoint): Double {
        val dx = max(0.0, max(HIVE_MIN - p.x, p.x - HIVE_MAX))
        val dy = max(0.0, max(HIVE_MIN - p.y, p.y - HIVE_MAX))
        return hypot(dx, dy)
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
