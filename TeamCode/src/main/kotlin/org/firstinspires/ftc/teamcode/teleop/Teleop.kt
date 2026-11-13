package org.firstinspires.ftc.teamcode.teleop

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.Gamepad

/** Gamepad 2 LT runs the selected shooter velocity (initially 1125 ticks/s); d-pad adjusts it. */
@TeleOp(name = "Teleop", group = "Teleop")
class Teleop : BaseTeleop() {
    override val manipulator: Gamepad get() = gamepad2
    override val initialShooterVelocity = 1125.0
}
