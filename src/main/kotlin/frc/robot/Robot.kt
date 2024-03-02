package frc.robot

import edu.wpi.first.wpilibj.TimedRobot
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.CommandScheduler
import org.littletonrobotics.junction.LoggedRobot


class Robot : LoggedRobot() {
    private lateinit var m_autonomousCommand: Command

    override fun robotInit() {
    }

    override fun robotPeriodic() {
        CommandScheduler.getInstance().run()
    }

    override fun disabledInit() {}

    override fun disabledPeriodic() {}

    override fun disabledExit() {}

    override fun autonomousInit() {
        RobotContainer.autonomousCommand.schedule()
    }

    override fun autonomousPeriodic() {}

    override fun autonomousExit() {}

    override fun teleopInit() {
        RobotContainer.autonomousCommand.cancel()

        RobotContainer.drivetrain.setDefaultCommand(RobotContainer.drivetrain.applyRequest {
            RobotContainer.drive.withVelocityX(
                1.5
            ).withVelocityY(0.0).withRotationalRate(0.0)
        })
    }

    override fun teleopPeriodic() {
    }

    override fun teleopExit() {}

    override fun testInit() {
        CommandScheduler.getInstance().cancelAll()
    }

    override fun testPeriodic() {}

    override fun testExit() {}

    override fun simulationPeriodic() {}
}
