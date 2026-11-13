package org.firstinspires.ftc.teamcode.teleop

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.Gamepad

/** One gamepad drives and runs the mechanisms; shooter velocity starts at 1125 ticks/s. */
@TeleOp(name = "One Man Army", group = "Teleop")
class OneManArmy : BaseTeleop() {
    override val manipulator: Gamepad get() = gamepad1
    override val initialShooterVelocity = 1125.0
}
