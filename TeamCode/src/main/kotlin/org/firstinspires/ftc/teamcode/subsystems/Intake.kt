package org.firstinspires.ftc.teamcode.subsystems

import com.pedropathing.ivy.Command
import com.pedropathing.ivy.commands.Commands.instant
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.HardwareNames

class Intake(hardwareMap: HardwareMap) {
    private val motor = hardwareMap.get(DcMotor::class.java, HardwareNames.INTAKE).apply {
        direction = DcMotorSimple.Direction.REVERSE
        mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        power = 0.0
    }

    fun run(power: Double): Command = instant { motor.power = power }.requiring(this)

    fun on(): Command = run(1.0)

    fun reverse(): Command = run(-1.0)

    fun off(): Command = run(0.0)
}
