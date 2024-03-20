package frc.robot.util

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard

object Telemetry {
    fun putNumber(name: String, value: Double, toggle: Boolean) {
        if (toggle) {
            SmartDashboard.putNumber(name, value)
        }
    }

    fun putBoolean(name: String, value: Boolean, toggle: Boolean) {
        if (toggle) {
            SmartDashboard.putBoolean(name, value)
        }
    }

    fun putString(name: String, value: String, toggle: Boolean) {
        if (toggle) {
            SmartDashboard.putString(name, value)
        }
    }
}