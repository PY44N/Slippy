package frc.robot.util

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser

class MiscUtils {
    fun <reified T: Enum> addAllEnumToSendable(sendable: SendableChooser<T>, options: Enum) {
        for (option in options.values()) {

        }
    }
}