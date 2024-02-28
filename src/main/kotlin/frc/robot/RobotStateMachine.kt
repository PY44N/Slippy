package frc.robot

import edu.wpi.first.math.geometry.Pose2d
import frc.robot.constants.CannonConstants


//This represents the desired shooter state
enum class ShooterState(val leftVel: Double, val rightVel: Double) {
    Stopped(0.0, 0.0),
    Prepped(CannonConstants.LEFT_SHOOTER_PREP_VELOCITY, CannonConstants.RIGHT_SHOOTER_PREP_VELOCITY),
    Shooting(CannonConstants.LEFT_SHOOTER_SHOOT_VELOCITY, CannonConstants.RIGHT_SHOOTER_SHOOT_VELOCITY)
}

//this represents the CURRENT note state
enum class NoteState {
    Empty,
    Intaking,
    Stored
}

//this represents the desired intake state (functionally current since intake immediately spins up)
enum class IntakeState(val innerPercent: Double, val outerPercent: Double) {
    Stopped(0.0, 0.0),
    Intaking(CannonConstants.INNER_INTAKE_PERCENT, CannonConstants.OUTER_INTAKE_PERCENT),
    Feeding(CannonConstants.INNER_FEED_PERCENT, CannonConstants.OUTER_FEED_PERCENT),
    Spitting(CannonConstants.INNER_SPIT_PERCENT, CannonConstants.OUTER_SPIT_PERCENT)
}

//this represents the desired trunk state
enum class TrunkState {
    Speaker,
    Amp,
    Stow,
    Trap,
    Floor,
    Source,
    Custom,
    Manual
}

enum class GlobalPosition {
    Shootable,
    Stage,
    NO,
    Source,
    Amp,
    Speaker
}

class RobotStateMachine {
    var trunkState: TrunkState = TrunkState.Stow;
    var intakeState: IntakeState = IntakeState.Stopped;
    var shooterState: ShooterState = ShooterState.Stopped;
    var noteState: NoteState = NoteState.Stored;

    var robotPosition: Pose2d = Pose2d()
        get() = RobotContainer.swerveSystem.swerveDrive.pose
        private set

    //Is the trunk at the desired position?
    var trunkReady: Boolean = false
        get() = TODO("Not yet implemented")
        private set

    //Is the shooter at the desired velocity?
    var shooterReady: Boolean = true
        get() = RobotContainer.cannonSystem.shooterReady()
        private set


    //Should be called in teleop periodic
    fun TeleopAutomaticStateManagement() {

    }
}