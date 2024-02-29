package frc.robot.util

import kotlin.math.abs
import kotlin.math.sign

class Math {
    companion object {
        fun wrapAroundAngles(angle: Double): Double {
            return ((abs(angle) % 360.0) * sign(angle) + 360.0) % 360
        }
    }
}