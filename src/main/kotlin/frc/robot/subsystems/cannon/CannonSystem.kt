package frc.robot.subsystems.cannon

import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.RobotContainer
import frc.robot.ShooterState

class CannonSystem(val io: CannonIO) : SubsystemBase() {
    class Shooter {
        fun kill() = Commands.runOnce({
            RobotContainer.stateMachine.shooterState = ShooterState.Stopped
        })
    }

    class Intake {

    }
}