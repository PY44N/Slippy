package frc.robot.commands

import com.pathplanner.lib.auto.AutoBuilder
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.DriveState
import frc.robot.RobotContainer
import frc.robot.constants.DriveConstants

class AutoDriveToPose(private val pose: Pose2d) : Command() {
    private var pathingCommand = AutoBuilder.pathfindToPose(
        pose,
        DriveConstants.PATHPLANNER_CONSTRAINTS,
        0.0, // Goal end velocity in meters/sec
        0.0 // Rotation delay distance in meters. This is how far the robot should travel before attempting to rotate.
    );


    override fun initialize() {
        RobotContainer.stateMachine.driveState = DriveState.Auto

        pathingCommand = AutoBuilder.pathfindToPose(
            pose,
            DriveConstants.PATHPLANNER_CONSTRAINTS,
            0.0, // Goal end velocity in meters/sec
            0.0 // Rotation delay distance in meters. This is how far the robot should travel before attempting to rotate.
        )

        pathingCommand.schedule()
    }

    override fun isFinished(): Boolean {
        println("Pathing Finished: ${pathingCommand.isFinished}")
        return pathingCommand.isFinished
    }

    override fun end(interrupted: Boolean) {
        RobotContainer.stateMachine.driveState = DriveState.Teleop
    }
}