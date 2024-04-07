package frc.robot.util

import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.robot.constants.DriveConstants


fun Trigger.betterToggleOnTrue(cmd: Command) {
    this.onTrue(Commands.runOnce({
        if (cmd.isScheduled) {
            cmd.cancel()
        } else {
            cmd.schedule()
        }
    }))
}

object ControllerUtil {
    //Takes in joystick inputs
    fun calculateJoyTranslation(
        rightX: Double,
        rightY: Double,
        throttle: Double,
        deadzoneX: Double,
        deadzoneY: Double
    ): Translation2d {
        return Translation2d(
            -MiscCalculations.calculateDeadzone(rightY, deadzoneX) * DriveConstants.MAX_SPEED * throttle,
            -MiscCalculations.calculateDeadzone(rightX, deadzoneY) * DriveConstants.MAX_SPEED * throttle
        )
    }

    // Milan: trust me bro this'll work totally definitely please don't question it
    fun calculateJoyThrottle(joyThrottle: Double) = (-joyThrottle + 1.0) / 2.0
}
