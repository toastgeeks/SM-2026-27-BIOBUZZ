package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple

/**
 * Spins one motor at a time so each port can be checked on the bench.
 *
 * Hold a button to run that motor at 30% power. Left bumper reverses it.
 */
@TeleOp(name = "Check: Motors", group = "Check")
class MotorCheck : LinearOpMode() {
    private data class MotorBinding(
        val label: String,
        val name: String,
        val pressed: () -> Boolean,
    )

    override fun runOpMode() {
        val bindings = listOf(
            MotorBinding("front left wheel (Control Hub 0)", HardwareNames.FRONT_LEFT) { gamepad1.a },
            MotorBinding("feed (Control Hub 1)", HardwareNames.FEED) { gamepad1.dpad_down },
            MotorBinding("back left wheel (Control Hub 2)", HardwareNames.BACK_LEFT) { gamepad1.y },
            MotorBinding("back right wheel (Control Hub 3)", HardwareNames.BACK_RIGHT) { gamepad1.b },
            MotorBinding("intake (Expansion Hub 1)", HardwareNames.INTAKE) { gamepad1.dpad_up },
            MotorBinding("right front wheel (Expansion Hub 2)", HardwareNames.RIGHT_FRONT) { gamepad1.x },
            MotorBinding("fly wheel (Expansion Hub 3)", HardwareNames.FLYWHEEL) { gamepad1.dpad_right },
        )
        val motors = bindings.map { binding ->
            hardwareMap.get(DcMotor::class.java, binding.name).apply {
                direction = if (binding.name == HardwareNames.INTAKE) {
                    DcMotorSimple.Direction.REVERSE
                } else {
                    DcMotorSimple.Direction.FORWARD
                }
                mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
                zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
                power = 0.0
            }
        }

        telemetry.addLine("Hold a button to spin that motor at 30%.")
        telemetry.addLine("Left bumper reverses the direction.")
        telemetry.addLine("A front left   Y back left   B back right   X right front")
        telemetry.addLine("Dpad up intake   Dpad down feed   Dpad right fly wheel")
        telemetry.update()

        waitForStart()

        while (opModeIsActive()) {
            val reverse = gamepad1.left_bumper
            val power = if (reverse) -POWER else POWER
            var active = "none"

            bindings.forEachIndexed { index, binding ->
                val running = binding.pressed()
                motors[index].power = if (running) power else 0.0
                if (running) {
                    active = binding.label
                }
            }

            telemetry.addData("Running", active)
            telemetry.addData("Direction", if (reverse) "reverse" else "forward")
            telemetry.addLine("A front left  Y back left  B back right  X right front")
            telemetry.addLine("Up intake  Down feed  Right fly wheel  LB reverse")
            telemetry.update()
        }
    }

    private companion object {
        const val POWER = 0.3
    }
}
