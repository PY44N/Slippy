package frc.robot.subsystems

import edu.wpi.first.math.geometry.Translation3d
import frc.robot.RobotContainer

class SwerveSystemIOReal : SwerveSystemIO {
    override fun updateInputs(inputs: SwerveSystemIO.SwerveSystemIOInputs) {
        inputs.robotAccel = RobotContainer.swerveSystem.swerveDrive.accel.orElse(Translation3d())
        inputs.robotVelocity = RobotContainer.swerveSystem.swerveDrive.robotVelocity
        inputs.gyroRotation = RobotContainer.swerveSystem.swerveDrive.gyroRotation3d
        inputs.robotPose = RobotContainer.swerveSystem.swerveDrive.pose

        for (i in 0..4) {
            inputs.drivePositions[i] =
                RobotContainer.swerveSystem.swerveDrive.modules[i].driveMotor.position
            inputs.driveVelocities[i] =
                RobotContainer.swerveSystem.swerveDrive.modules[i].driveMotor.velocity

            inputs.turnAbsolutePositions[i] =
                RobotContainer.swerveSystem.swerveDrive.modules[i].absolutePosition
            inputs.turnPositions[i] =
                RobotContainer.swerveSystem.swerveDrive.modules[i].relativePosition
            inputs.turnVelocities[i] =
                RobotContainer.swerveSystem.swerveDrive.modules[i].angleMotor.velocity
        }
    }
}