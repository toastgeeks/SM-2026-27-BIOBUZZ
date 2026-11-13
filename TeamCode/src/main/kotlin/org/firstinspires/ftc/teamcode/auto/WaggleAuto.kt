package org.firstinspires.ftc.teamcode.auto

import com.pedropathing.api.Paths.curve
import com.pedropathing.api.Paths.line
import com.pedropathing.math.Pose
import com.pedropathing.math.Vector2D
import com.pedropathing.paths.Path
import com.pedropathing.paths.interpolator.Interpolator
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.pedro.Constants
import kotlin.math.PI
import kotlin.math.sin

/**
 * BIOBUZZ drivetrain-only auto: a honeybee "waggle dance", an orbit around our HIVE, then PARK.
 *
 * Field frame (red view): origin in the red/audience corner, x toward the blue ALLIANCE AREA,
 * y toward the far wall, heading 0 facing blue. Blue runs the same route rotated 180 degrees
 * about the field center, which matches how the BIOBUZZ field is laid out.
 *
 * Red route, all in columns A-C (red side during AUTO):
 *  1. Start in tile B1, back against the audience wall, facing the far wall.
 *  2. Waggle run: drive about 28 in toward the far wall while swinging heading +/-15 degrees.
 *  3. Orbit: strafe a 90 degree arc (50 in radius) around the HIVE, always facing it.
 *  4. Park: slide into the red LOADING ZONE (left wall, row 5) and hold. LEAVE 3 + PARK 5.
 */
abstract class WaggleAuto(private val blue: Boolean) : LinearOpMode() {

    /** Converts a red-frame pose to this alliance's pose. */
    private fun pose(x: Double, y: Double, heading: Double): Pose =
        if (blue) Pose(FIELD - x, FIELD - y, heading + PI) else Pose(x, y, heading)

    private fun point(x: Double, y: Double): Vector2D =
        if (blue) Vector2D.cartesian(FIELD - x, FIELD - y) else Vector2D.cartesian(x, y)

    override fun runOpMode() {
        val follower = Constants.create(hardwareMap)

        val start = pose(START_X, START_Y, PI / 2)
        val arcStart = pose(36.6, 36.6, PI / 2)
        val arcEnd = pose(36.6, 107.4, -PI / 4)
        val park = pose(16.0, 108.0, 0.0)
        val hive = point(HIVE_X, HIVE_Y)
        val baseHeading = start.heading()

        // Heading swings side to side while driving straight, like a bee's waggle run.
        val waggle: Path = line(start, arcStart).heading(Interpolator { _, t ->
            baseHeading + WAGGLE_AMPLITUDE * sin(2 * PI * WAGGLE_CYCLES * t)
        })

        // Cubic Bezier approximating a 90 degree arc around the HIVE, from 225 to 135 degrees.
        val orbit: Path = curve(
            arcStart,
            pose(17.1, 56.1, 0.0),
            pose(17.1, 87.9, 0.0),
            arcEnd,
        ).facingPoint(hive)

        val parkPath: Path = line(arcEnd, park).linear(arcEnd, park)

        val legs = listOf("Waggle" to waggle, "Orbit" to orbit, "Park" to parkPath)

        follower.setPose(start)
        telemetry.addLine(if (blue) "BLUE: tile E6, back on the far wall, facing the audience."
            else "RED: tile B1, back on the audience wall, facing the far wall.")
        telemetry.addLine("Waggle, orbit the HIVE, then PARK in the LOADING ZONE.")
        telemetry.update()

        waitForStart()
        follower.setPose(start)

        var leg = 0
        follower.follow(legs[leg].second)

        try {
            while (opModeIsActive()) {
                follower.update()

                if (leg < legs.size && follower.atParametricEnd()) {
                    leg++
                    if (leg < legs.size) follower.follow(legs[leg].second) else follower.hold(park)
                }

                val current = follower.pose()
                telemetry.addData("Leg", if (leg < legs.size) legs[leg].first else "Parked (holding)")
                telemetry.addData("X (in)", "%.1f", current.x())
                telemetry.addData("Y (in)", "%.1f", current.y())
                telemetry.addData("Heading (deg)", "%.1f", Math.toDegrees(current.heading()))
                telemetry.update()
            }
        } finally {
            follower.stop()
        }
    }

    companion object {
        const val FIELD = 144.0
        const val HIVE_X = 72.0
        const val HIVE_Y = 72.0

        /** Tile B1 center line. */
        const val START_X = 36.0

        /** Half the robot's length front to back, so its back touches the wall. */
        const val START_Y = 9.0

        const val WAGGLE_AMPLITUDE = 15.0 * PI / 180.0
        const val WAGGLE_CYCLES = 2.0
    }
}

@Autonomous(name = "Waggle Red", group = "Auto")
class WaggleRed : WaggleAuto(blue = false)

@Autonomous(name = "Waggle Blue", group = "Auto")
class WaggleBlue : WaggleAuto(blue = true)
