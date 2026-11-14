package org.firstinspires.ftc.teamcode.auto

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.auto.route.*

/**
 * Start in D6 facing the far wall and shoot, then go down the right side of the HIVE and PARK in
 * F2.
 */
@Autonomous(name = "Loading Zone Park Blue", group = "Auto")
class LoadingZoneParkBlue : RouteAuto(
    route(start = D6 against FAR, facing = FAR) {
        spinUp(velocity = 1137)
        shoot(feedSeconds = 3)
        driveTo(at(E + 0.5, 5.5))
        driveTo(at(E + 0.5, 2), facing = RED)
        driveTo(F2.offWall(BLUE, gap = 6)) // touching the wall at the end is a penalty
    },
)
