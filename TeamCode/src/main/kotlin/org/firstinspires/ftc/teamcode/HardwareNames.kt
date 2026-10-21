package org.firstinspires.ftc.teamcode

/**
 * Device names from the Control Hub configuration "bi2w".
 *
 * Control Hub drivetrain
 *   motor 0: backLeftMotor
 *   motor 1: backRightMotor
 *   motor 2: frontLeftMotor
 *   motor 3: frontRightMotor
 *   I2C bus 1: pinpoint
 *
 * Expansion Hub
 *   motor 0: flywheel1
 *   motor 1: flywheel2
 *   motor 2: feed
 *   motor 3: intake
 *
 * Both flywheels use Direction.REVERSE. They are not opposing.
 * flywheel1 has an encoder on Expansion Hub port 0. flywheel2 does not.
 */
object HardwareNames {
    const val FRONT_LEFT = "frontLeftMotor"
    const val RIGHT_FRONT = "frontRightMotor"
    const val BACK_LEFT = "backLeftMotor"
    const val BACK_RIGHT = "backRightMotor"
    const val FLYWHEEL = "flywheel1"
    const val FLYWHEEL_2 = "flywheel2"
    const val FEED = "feed"
    const val INTAKE = "intake"
    const val PINPOINT = "pinpoint"
}
