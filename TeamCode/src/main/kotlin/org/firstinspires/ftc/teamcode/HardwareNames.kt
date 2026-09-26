package org.firstinspires.ftc.teamcode

/**
 * Device names from the Control Hub configuration "bi2w".
 *
 * Control Hub
 *   motor 0: front left wheel
 *   motor 1: feed
 *   motor 2: back left wheel
 *   motor 3: back right wheel
 *   I2C bus 1: pinpoint
 *
 * Expansion Hub
 *   motor 1: intake
 *   motor 2: right front wheel
 *   motor 3: fly wheel
 *
 * fly wheel 2 is reversed in code relative to fly wheel.
 */
object HardwareNames {
    const val FRONT_LEFT = "front left wheel"
    const val FEED = "feed"
    const val BACK_LEFT = "back left wheel"
    const val BACK_RIGHT = "back right wheel"
    const val INTAKE = "intake"
    const val RIGHT_FRONT = "right front wheel"
    const val FLYWHEEL = "fly wheel"
    const val FLYWHEEL_2 = "fly wheel 2"
    const val PINPOINT = "pinpoint"
}
