package frc.robot.subsystems

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation3d
import edu.wpi.first.math.geometry.Translation3d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import frc.robot.RobotContainer
import org.littletonrobotics.junction.LogTable
import org.littletonrobotics.junction.inputs.LoggableInputs

class SwerveSystemIO {
    object SwerveSystemIOInputs : LoggableInputs {
        var robotAccel: Translation3d = Translation3d()
        var robotVelocity: ChassisSpeeds = ChassisSpeeds()
        var gyroRotation: Rotation3d = Rotation3d()
        var robotPose: Pose2d = Pose2d()

        var drivePositions: DoubleArray = doubleArrayOf(0.0, 0.0, 0.0, 0.0)
        var driveVelocities: DoubleArray = doubleArrayOf(0.0, 0.0, 0.0, 0.0)

        var turnAbsolutePositions: DoubleArray = doubleArrayOf(0.0, 0.0, 0.0, 0.0)
        var turnPositions: DoubleArray = doubleArrayOf(0.0, 0.0, 0.0, 0.0)
        var turnVelocities: DoubleArray = doubleArrayOf(0.0, 0.0, 0.0, 0.0)

        override fun toLog(table: LogTable) {
            table.put("robotAccel", robotAccel)
            table.put("robotVelocity", robotVelocity)
            table.put("gyroRotation", gyroRotation)
            table.put("robotPose", robotPose)

            table.put("drivePositions", drivePositions)
            table.put("driveVelocities", driveVelocities)

            table.put("turnAbsolutePositions", turnAbsolutePositions)
            table.put("turnPositions", turnPositions)
            table.put("turnVelocities", turnVelocities)
        }

        override fun fromLog(table: LogTable) {
            robotAccel = table.get("robotAccel", robotAccel)[0]
            robotVelocity = table.get("robotVelocity", robotVelocity)[0]
            gyroRotation = table.get("gyroRotation", gyroRotation)[0]
            robotPose = table.get("robotPose", robotPose)[0]

            drivePositions = table.get("drivePositions", drivePositions)
            driveVelocities = table.get("driveVelocities", driveVelocities)

            turnAbsolutePositions = table.get("turnAbsolutePositions", turnAbsolutePositions)
            turnPositions = table.get("turnPositions", turnPositions)
            turnVelocities = table.get("turnVelocities", turnVelocities)
        }
    }

    fun updateInputs(inputs: SwerveSystemIOInputs) {
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