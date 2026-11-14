package org.firstinspires.ftc.teamcode.auto

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.auto.route.*

/**
 * Start in C6 with the back on the far wall, facing the audience. Wait 8 s, shoot, then PARK in
 * A5.
 */
@Autonomous(name = "Far Start Park Red", group = "Auto")
class FarStartParkRed : RouteAuto(
    route(start = C6 against FAR, facing = FAR) {
        wait(5000)
        shoot(velocity = 1137, feedSeconds = 3)
        driveTo(C5, facing = RED)
        // Halfway between rows 5 and 6. Touching the wall at the end is a penalty.
        driveTo((A / R5_6).offWall(RED, gap = 6))
    },
)
