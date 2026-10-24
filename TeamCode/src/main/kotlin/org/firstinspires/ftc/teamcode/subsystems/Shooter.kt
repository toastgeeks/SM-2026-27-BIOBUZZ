package org.firstinspires.ftc.teamcode.subsystems

import com.acmerobotics.dashboard.config.Config
import com.pedropathing.ivy.Command
import com.pedropathing.ivy.commands.Commands.instant
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.PIDFCoefficients
import org.firstinspires.ftc.teamcode.HardwareNames

/**
 * Both flywheels share a direction and a power. They are not opposing.
 * Both motors have encoders; reported position and velocity come from flywheel1.
 * With no [pidf], both motors use open-loop power as before. Supplying coefficients
 * enables velocity control with the same PIDF coefficients on both motors.
 */
@Config
class Shooter(hardwareMap: HardwareMap, private val pidf: PIDFCoefficients? = null) {
    private val flywheel1 = flywheel(hardwareMap, HardwareNames.FLYWHEEL).apply {
        mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
    }
    private val flywheel2 = flywheel(hardwareMap, HardwareNames.FLYWHEEL_2)

    init {
        if (pidf != null) {
            flywheel1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf)
            flywheel2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf)
            flywheel1.mode = DcMotor.RunMode.RUN_USING_ENCODER
            flywheel2.mode = DcMotor.RunMode.RUN_USING_ENCODER
        }
    }

    /** flywheel1 encoder ticks. */
    val position: Int get() = flywheel1.currentPosition

    /** flywheel1 encoder ticks per second. */
    val velocity: Double get() = flywheel1.velocity

    /** flywheel2 encoder ticks per second. */
    val flywheel2Velocity: Double get() = flywheel2.velocity

    /** In PIDF mode, [power] is a fraction of the configured maximum motor velocity. */
    fun run(power: Double): Command = instant {
        flywheel1.power = power
        flywheel2.power = power
    }.requiring(this)

    /** Commands both motors in encoder ticks per second. Requires PIDF at construction. */
    fun runVelocity(ticksPerSecond: Double): Command = instant {
        check(pidf != null) { "Supply PIDF coefficients to use shooter velocity control." }
        flywheel1.velocity = ticksPerSecond
        flywheel2.velocity = ticksPerSecond
    }.requiring(this)

    /** Updates both velocity controllers without changing their target or run mode. */
    fun setPIDF(coefficients: PIDFCoefficients) {
        check(pidf != null) { "Supply PIDF coefficients to use shooter velocity control." }
        flywheel1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, coefficients)
        flywheel2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, coefficients)
    }

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

    companion object {
        @JvmField @Volatile var PIDF_P = 1.25
        @JvmField @Volatile var PIDF_I = 0.0
        @JvmField @Volatile var PIDF_D = 0.0
        @JvmField @Volatile var PIDF_F = 24.0
        @JvmField @Volatile var TARGET = 800.0
        @JvmField @Volatile var TARGET_STEP = 25.0

        /** A fresh copy keeps callers from modifying the shared tuning values. */
        val PIDF_COEFFICIENTS: PIDFCoefficients
            get() = PIDFCoefficients(PIDF_P, PIDF_I, PIDF_D, PIDF_F)
    }
}
