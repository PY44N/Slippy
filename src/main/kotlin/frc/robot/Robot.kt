package frc.robot

import edu.wpi.first.wpilibj.TimedRobot
import com.revrobotics.CANSparkLowLevel
import com.revrobotics.CANSparkMax
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.wpilibj.PowerDistribution
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
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
        RobotContainer.teleopSwerveCommand.schedule()

//        RobotContainer.drivetrain.setDefaultCommand(RobotContainer.drivetrain.applyRequest {
//            RobotContainer.drive.withVelocityX(
//                    1.5
//            ).withVelocityY(0.0).withRotationalRate(0.0)
//        })
    }

    override fun teleopPeriodic() {
        RobotContainer.stateMachine.TeleopAutomaticStateManagement()

        //        SmartDashboard.putNumber("JoyX", RobotContainer.rightJoystick.x)
        //        SmartDashboard.putNumber("JoyY", RobotContainer.rightJoystick.y)
        //        SmartDashboard.putNumber("JoyTwist", RobotContainer.rightJoystick.twist)
        //        val desiredState = SwerveModuleState(0.0, Rotation2d(0.0, 0.0))
        //        if(RobotContainer.xboxController.b().asBoolean)
        //            RobotContainer.trunkSystem.goManual()
        //        if(RobotContainer.xboxController.x().asBoolean)
        //            RobotContainer.trunkSystem.STOP()
        //        if(RobotContainer.xboxController.a().asBoolean)
        //            RobotContainer.trunkSystem.calibrate()
        //        RobotContainer.trunkSystem.elevate(-RobotContainer.xboxController.leftY)
        //        RobotContainer.trunkSystem.rotate(-RobotContainer.xboxController.rightY*.3)

    }

    override fun teleopExit() {}

    override fun testInit() {
        CommandScheduler.getInstance().cancelAll()
    }

    override fun testPeriodic() {
        //        RobotContainer.swerveSystem.drive(Translation2d(0.25, 0.0), 0.0, true)
        //        val calibrator = ShooterCalibrator("/u/shooter_calibrator/test1.csv");
        //        calibrator.writeOut(shots)
        //        val readShots = calibrator.readCsv();
        //        readShots.forEach {
        //            println(it.toCSV())
        //        }
    }

    override fun testExit() {}

    override fun simulationPeriodic() {}
}
