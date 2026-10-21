package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
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
            MotorBinding("back left (Control Hub 0)", HardwareNames.BACK_LEFT) { gamepad1.y },
            MotorBinding("back right (Control Hub 1)", HardwareNames.BACK_RIGHT) { gamepad1.b },
            MotorBinding("front left (Control Hub 2)", HardwareNames.FRONT_LEFT) { gamepad1.a },
            MotorBinding("front right (Control Hub 3)", HardwareNames.RIGHT_FRONT) { gamepad1.x },
            MotorBinding("flywheel1 (Expansion Hub 0)", HardwareNames.FLYWHEEL) { gamepad1.dpad_right },
            MotorBinding("flywheel2 (Expansion Hub 1)", HardwareNames.FLYWHEEL_2) { gamepad1.dpad_left },
            MotorBinding("feed (Expansion Hub 2)", HardwareNames.FEED) { gamepad1.dpad_down },
            MotorBinding("intake (Expansion Hub 3)", HardwareNames.INTAKE) { gamepad1.dpad_up },
        )
        val motors = bindings.map { binding ->
            hardwareMap.get(DcMotorEx::class.java, binding.name).apply {
                direction = when (binding.name) {
                    HardwareNames.INTAKE,
                    HardwareNames.FLYWHEEL,
                    HardwareNames.FLYWHEEL_2 -> DcMotorSimple.Direction.REVERSE
                    else -> DcMotorSimple.Direction.FORWARD
                }
                mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
                zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
                power = 0.0
            }
        }
        val flywheel1 = motors[bindings.indexOfFirst { it.name == HardwareNames.FLYWHEEL }]
        flywheel1.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        flywheel1.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER

        telemetry.addLine("Hold a button to spin that motor at 30%.")
        telemetry.addLine("Left bumper reverses the direction.")
        telemetry.addLine("A front left   Y back left   B back right   X right front")
        telemetry.addLine("Dpad up intake   Dpad down feed   Dpad right flywheel1   Dpad left flywheel2")
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
            telemetry.addData("Flywheel1 pos", flywheel1.currentPosition)
            telemetry.addData("Flywheel1 ticks/s", "%.0f", flywheel1.velocity)
            telemetry.addLine("A front left  Y back left  B back right  X right front")
            telemetry.addLine("Up intake  Down feed  Right flywheel1  Left flywheel2  LB reverse")
            telemetry.update()
        }
    }

    private companion object {
        const val POWER = 0.3
    }
}
