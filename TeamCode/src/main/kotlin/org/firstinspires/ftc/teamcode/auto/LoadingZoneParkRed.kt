package org.firstinspires.ftc.teamcode.auto

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.auto.route.*

/**
 * Start in C1 facing the audience wall and shoot, then go up the left side of the HIVE and PARK
 * in A5.
 */
@Autonomous(name = "Loading Zone Park Red", group = "Auto")
class LoadingZoneParkRed : RouteAuto(
    route(start = C1 against AUDIENCE, facing = AUDIENCE) {
        spinUp(velocity = 1137)
        shoot(feedSeconds = 3)
        wait(5000)
        driveTo(at(A + 0.5, 1.5))
        driveTo(at(A + 0.5, 5), facing = BLUE)
        driveTo(A5.offWall(RED, gap = 6)) // touching the wall at the end is a penalty
    },
)
