package org.firstinspires.ftc.teamcode.subsystems

import com.pedropathing.ivy.Command
import com.pedropathing.ivy.commands.Commands.instant
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.HardwareNames

/**
 * Both flywheels share a direction and a power. They are not opposing.
 * Only flywheel1 has an encoder, so position and velocity come from it.
 */
class Shooter(hardwareMap: HardwareMap) {
    private val flywheel1 = flywheel(hardwareMap, HardwareNames.FLYWHEEL).apply {
        mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
    }
    private val flywheel2 = flywheel(hardwareMap, HardwareNames.FLYWHEEL_2)

    /** flywheel1 encoder ticks. */
    val position: Int get() = flywheel1.currentPosition

    /** flywheel1 encoder ticks per second. */
    val velocity: Double get() = flywheel1.velocity

    fun run(power: Double): Command = instant {
        flywheel1.power = power
        flywheel2.power = power
    }.requiring(this)

    fun on(): Command = run(1.0)

    fun reverse(): Command = run(-1.0)

    fun off(): Command = run(0.0)

    private fun flywheel(hardwareMap: HardwareMap, name: String): DcMotorEx {
        return hardwareMap.get(DcMotorEx::class.java, name).apply {
            direction = DcMotorSimple.Direction.REVERSE
            mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
            zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
            power = 0.0
        }
    }
}
