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
        spinUp(velocity = 1137)
        wait(5000) // the flywheels reach speed during the wait
        shoot(feedSeconds = 3)
        driveTo(C5, facing = RED)
        // Halfway between rows 5 and 6. Touching the wall at the end is a penalty.
        driveTo(at(A, 5.5).offWall(RED, gap = 6))
    },
)
