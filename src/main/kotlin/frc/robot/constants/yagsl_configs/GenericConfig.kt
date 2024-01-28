package frc.robot.constants.yagsl_configs

import swervelib.parser.SwerveControllerConfiguration
import swervelib.parser.SwerveDriveConfiguration

class YAGSLConfig(
    val driveConfig: SwerveDriveConfiguration,
    val controllerConfig: SwerveControllerConfiguration,
    val maxSpeedMPS: Double,
)