package frc.robot

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.commands.trunk.CalibrateTrunk
import frc.robot.constants.CannonConstants
import frc.robot.constants.FieldPositions
import frc.robot.constants.TrunkConstants
import frc.robot.util.Telemetry

//This represents the desired shooter state
enum class ShooterState(val shooterVel: Double) {
    Stopped(0.0),
    Prepped(CannonConstants.SHOOTER_PREP_VELOCITY),
    Shooting(CannonConstants.SHOOTER_SHOOT_VELOCITY),
    Amping(1500.0),
    CloseShooting(3650.0),
}

//this represents the CURRENT note state
enum class NoteState {
    Empty,
    Intaking,
    Stored
}

//this represents the DESIRED intake state (functionally current since intake immediately spins up)
enum class IntakeState(var innerPercent: Double, var outerPercent: Double) {
    Stopped(0.0, 0.0),
    Intaking(CannonConstants.INNER_INTAKE_PERCENT, CannonConstants.OUTER_INTAKE_PERCENT),
    Feeding(CannonConstants.INNER_FEED_PERCENT, CannonConstants.OUTER_FEED_PERCENT),
    Spitting(CannonConstants.INNER_SPIT_PERCENT, CannonConstants.OUTER_SPIT_PERCENT),
    AmpSpitting(CannonConstants.INNER_AMP_PERCENT, CannonConstants.OUTER_AMP_PERCENT),
    }

//this represents the DESIRED trunk state
enum class TrunkPose(var angle: Double, var position: Double) {
    AMP_GOING(TrunkConstants.AMP_ANGLE, 0.3),
    AMP(TrunkConstants.AMP_ANGLE, TrunkConstants.AMP_POSITION),
    AMP_SHOOTING(50.0, 0.4),
    AMP_LEAVING(TrunkConstants.AMP_ANGLE - 10.0, TrunkConstants.STOW_POSITION),
    SPEAKER(TrunkConstants.STOW_ANGLE, TrunkConstants.STOW_POSITION),
    INTAKE(TrunkConstants.INTAKE_ANGLE, TrunkConstants.INTAKE_POSITION),
    INTAKE_PREP(TrunkConstants.SAFE_TRAVEL_ANGLE, TrunkConstants.SAFE_TO_DROP_INTAKE_POSITION),
    STOW(TrunkConstants.STOW_ANGLE, TrunkConstants.STOW_POSITION),
    HIGH_STOW(TrunkConstants.HIGH_STOW_ANGLE, TrunkConstants.STOW_POSITION),
    TRAP(TrunkConstants.TRAP_ANGLE, TrunkConstants.TRAP_POSITION),
    CalibrationAngle(90.0, TrunkConstants.STOW_POSITION),
    CLIMB(180.0, TrunkConstants.STOW_POSITION),
    CLIMB_STAGE_1(180.0, .23), // We need to make it so we can do positions that aren't in this enum
    CLIMB_STAGE_2(83.0, .23),
    CLIMB_STAGE_FINAL(78.0, .31)
}

//CURRENT states
enum class GlobalZones(val range: Pair<Translation2d, Translation2d>) {
    Wing(Pair(FieldPositions.WING_START, FieldPositions.WING_END)),
    Stage(Pair(FieldPositions.STAGE_START, FieldPositions.STAGE_END)),
    NO(Pair(FieldPositions.NO_START, FieldPositions.NO_END)),
}

//DESIRED states
enum class GlobalSpecificPosition(val pos: Translation2d) {
    Source(FieldPositions.SOURCE),
    Amp(FieldPositions.AMP),
    Speaker(FieldPositions.SPEAKER)
}

//What do we want to do with our current note (or lack of note)?
enum class RobotAction() {
    Amp,
    Speaker,
    Trap,
    FloorIntake,
    SourceIntake,
    Chill
}

enum class DriveState() {
    Teleop,
    TranslationTeleop,
    Auto
}

enum class AutoStateManagement {
    Enabled,
    Disabled
}

enum class ShootPosition(val position: Pose2d) {
    AutoAim(Pose2d()),
    StageFront(Pose2d(Translation2d(2.75, 4.0), Rotation2d(-32.0))), // Angle might not be measured correctly
    StageSide(Pose2d(Translation2d(3.36, 4.83), Rotation2d(-32.0))) // Angle might not be measured correctly
}

class RobotStateMachine {

    var intakeState: IntakeState = IntakeState.Stopped
    var shooterState: ShooterState = ShooterState.Stopped
    var noteState: NoteState = NoteState.Stored

    var currentTrunkCommandLocked = false

    var currentTrunkCommand: Command = CalibrateTrunk()
        set(value) {
            if (!currentTrunkCommandLocked) {
                field.cancel()
                field = value
                field.schedule()
            }
        }

    var currentRobotZone: GlobalZones = GlobalZones.Wing
    var prevRobotZone: GlobalZones = GlobalZones.Wing

    val robotAction: RobotAction
        get() = if (RobotContainer.robotActionSendable.selected != null) {
            RobotContainer.robotActionSendable.selected
        } else {
            RobotAction.Chill
        }
    val shootPosition: ShootPosition = ShootPosition.StageFront
    var driveState: DriveState = DriveState.Teleop

    var autoStateManagement: AutoStateManagement = AutoStateManagement.Disabled

    fun logStates() {
        RobotContainer.telemetry.stateMachineTelemetry =
                SmartDashboard.getBoolean("State Machine Telemetry", RobotContainer.telemetry.stateMachineTelemetry)
        SmartDashboard.putBoolean("State Machine Telemetry", RobotContainer.telemetry.stateMachineTelemetry)

        Telemetry.putString("Note State", noteState.name, RobotContainer.telemetry.stateMachineTelemetry)
        Telemetry.putString("Shooter State", shooterState.name, RobotContainer.telemetry.stateMachineTelemetry)
    }

    //Is the trunk at the desired position?
    val trunkReady: Boolean
        get() = RobotContainer.trunkSystem.isAtPose

    //Is the shooter at the desired velocity?
    val shooterReady: Boolean
        get() = RobotContainer.cannonSystem.shooterReady()

    //Should be called in teleop periodic
    fun TeleopAutomaticStateManagement() {
//        if (autoStateManagement != AutoStateManagement.Enabled) {
//            return
//        }
//        if (currentRobotZone != prevRobotZone) {
//            when (currentRobotZone) {
//                //when in NO set to stow and sping down shooter (if not overridden)
//                GlobalZones.NO -> {
//                    if (noteState == NoteState.Empty) {
//                        trunkState = TrunkState.Stow
//                        shooterState = ShooterState.Stopped
//                    }
//                }
//                //when in wing set to wing shooting position and enable the auto (pivot) aiming and prep shooter (if not manual override)
//                GlobalZones.Wing -> {
//                    if (noteState != NoteState.Empty) {
//                        if (robotAction == RobotAction.Speaker) {
//                            trunkState = TrunkState.Speaker
//                            shooterState = ShooterState.Prepped
//                        }
//                    }
//                }
//                //when in stage, set to stage shooting position and enable the auto (pivot) aiming and prep shooter (if not manual override)
//                GlobalZones.Stage -> {
//                    if (noteState != NoteState.Empty) {
//                        if (robotAction == RobotAction.Speaker) {
//                            trunkState = TrunkState.SpeakerFromStage
//                            shooterState = ShooterState.Prepped
//                        }
//                    }
//                }
//            }
//        }
    }
}
