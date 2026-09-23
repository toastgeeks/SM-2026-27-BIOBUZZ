package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp

/**
 * Minimal Kotlin OpMode. Appears on the Driver Station as "Hello World".
 */
@TeleOp(name = "Hello World", group = "Test")
class HelloWorld : LinearOpMode() {
    override fun runOpMode() {
        telemetry.addData("Status", "Hello World")
        telemetry.update()

        waitForStart()

        while (opModeIsActive()) {
            telemetry.addData("Status", "Hello World")
            telemetry.update()
        }
    }
}
