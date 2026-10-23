package org.firstinspires.ftc.teamcode.teleop

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.Gamepad

/** Gamepad 1 drives. Gamepad 2 runs the intake, flywheels, and feed. */
@TeleOp(name = "Teleop", group = "Teleop")
class Teleop : BaseTeleop() {
    override val manipulator: Gamepad get() = gamepad2
}
