package org.firstinspires.ftc.teamcode.subsystems

import com.pedropathing.ivy.Command
import com.pedropathing.ivy.commands.Commands.instant
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.HardwareNames

class Feed(hardwareMap: HardwareMap) {
    private val motor = hardwareMap.get(DcMotor::class.java, HardwareNames.FEED).apply {
        direction = DcMotorSimple.Direction.FORWARD
        mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        power = 0.0
    }

    fun on(): Command = instant { motor.power = 1.0 }.requiring(this)

    fun reverse(): Command = instant { motor.power = -1.0 }.requiring(this)

    fun off(): Command = instant { motor.power = 0.0 }.requiring(this)
}
