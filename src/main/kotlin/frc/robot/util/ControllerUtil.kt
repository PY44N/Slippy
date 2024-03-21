package frc.robot.util

import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.button.Trigger

object ControllerUtil {
    fun betterToggleOnTrue(button: Trigger, cmd: Command) {
        button.onTrue(Commands.runOnce({
            if (cmd.isScheduled) {
                cmd.cancel()
            } else {
                cmd.schedule()
            }
        }))
    }
}