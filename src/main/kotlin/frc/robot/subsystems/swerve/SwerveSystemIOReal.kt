package frc.robot.subsystems.swerve

import frc.robot.RobotContainer
import frc.robot.subsystems.swerve.SwerveSystemIO
import swervelib.SwerveController

class SwerveSystemIOReal : SwerveSystemIO {
    override fun updateInputs(inputs: SwerveSystemIO.SwerveSystemIOInputs) {
//        inputs.givenTranslation =
//            SwerveController.getTranslation2d(RobotContainer.swerveSystem.swerveDrive.robotVelocity)
//        inputs.givenRotation = RobotContainer.swerveSystem.inputRotation
    }
}