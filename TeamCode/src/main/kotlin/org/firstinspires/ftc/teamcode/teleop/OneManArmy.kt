package org.firstinspires.ftc.teamcode.teleop

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.Gamepad

/** One gamepad drives and runs the intake, flywheels, and feed. */
@TeleOp(name = "One Man Army", group = "Teleop")
class OneManArmy : BaseTeleop() {
    override val manipulator: Gamepad get() = gamepad1
}
