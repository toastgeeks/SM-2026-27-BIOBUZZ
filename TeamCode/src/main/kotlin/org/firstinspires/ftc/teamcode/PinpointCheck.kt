package org.firstinspires.ftc.teamcode

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D

/**
 * Reads the goBILDA Pinpoint on Control Hub I2C bus 1.
 * Inches are uncalibrated until pod offsets and resolution are set.
 */
@TeleOp(name = "Check: Pinpoint", group = "Check")
class PinpointCheck : LinearOpMode() {
    override fun runOpMode() {
        val pinpoint = hardwareMap.get(GoBildaPinpointDriver::class.java, HardwareNames.PINPOINT)

        telemetry.addLine("Push the robot to see the pose change.")
        telemetry.addLine("A zeros the pose. X recalibrates the IMU. Hold still for X.")
        telemetry.update()
        waitForStart()

        while (opModeIsActive()) {
            if (gamepad1.a) {
                pinpoint.setPosition(Pose2D(DistanceUnit.INCH, 0.0, 0.0, AngleUnit.DEGREES, 0.0))
            }
            if (gamepad1.x) {
                pinpoint.recalibrateIMU()
            }

            pinpoint.update()

            telemetry.addData("Status", pinpoint.deviceStatus)
            telemetry.addData("Device", "%d  v%d", pinpoint.deviceID, pinpoint.deviceVersion)
            telemetry.addData("Encoder X", pinpoint.encoderX)
            telemetry.addData("Encoder Y", pinpoint.encoderY)
            telemetry.addData("X (in)", pinpoint.getPosX(DistanceUnit.INCH))
            telemetry.addData("Y (in)", pinpoint.getPosY(DistanceUnit.INCH))
            telemetry.addData("Heading (deg)", pinpoint.getHeading(AngleUnit.DEGREES))
            telemetry.addData("Frequency", pinpoint.frequency)
            telemetry.update()
        }
    }
}
