package frc.robot.subsystems.cannon

import MiscCalculations
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.*
import frc.robot.constants.CannonConstants

class CannonSystem(private val io: CannonIO) : SubsystemBase() {

    //Desired shooter velocities
    private var desiredLeftVel = 0.0
    private var desiredRightVel = 0.0

    //Desired intake percentages
    private var desiredOuterPercent = 0.0
    private var desiredInnerPercent = 0.0


    private var exitBreakBeamTriggerTime: Double = -1.0; //

    init {

    }


    fun killShooter() {
        io.setLeftShooter(0.0)
        io.setRightShooter(0.0)
        RobotContainer.stateMachine.shooterState = ShooterState.Stopped
    }

    fun prep() {
        RobotContainer.stateMachine.shooterState = ShooterState.Prepped;
    }

    fun shoot() {
        RobotContainer.stateMachine.shooterState = ShooterState.Shooting;
        println("called cannonsystem.shoot")

        //TODO: remove this
//        exitBreakBeamTriggerTime = Timer.getFPGATimestamp()
    }


    fun ampSpit() {
        RobotContainer.stateMachine.intakeState = IntakeState.AmpSpitting
    }


    fun intake() {
        RobotContainer.stateMachine.intakeState = IntakeState.Intaking
    }

    fun spit() {
        RobotContainer.stateMachine.intakeState = IntakeState.Spitting
    }

    fun feed() {
        RobotContainer.stateMachine.intakeState = IntakeState.Feeding
    }

    fun killIntake() {
        RobotContainer.stateMachine.intakeState = IntakeState.Stopped
    }

    fun shooterReady(): Boolean {
        //Shooter up to speed
        return MiscCalculations.appxEqual(RobotContainer.stateMachine.shooterState.leftVel, io.getLeftShooterVel(), CannonConstants.SHOOTER_VELOCITY_DEADZONE) && MiscCalculations.appxEqual(RobotContainer.stateMachine.shooterState.rightVel, io.getRightShooterVel(), CannonConstants.SHOOTER_VELOCITY_DEADZONE)
    }

    override fun periodic() {
        SmartDashboard.putBoolean("Stow Beam Break", io.getLoadedBeamBreak())
        SmartDashboard.putNumber("Left Cannon Speed", io.getLeftShooterVel())


        /*--------------------
             Beam Breaks
        -----------------------*/

        //Note is shot
//        if (io.getExitBeamBreak()) {
//            exitBreakBeamTriggerTime = Timer.getFPGATimestamp()
//        }
        //Note is stored
        /*else*/ if (io.getLoadedBeamBreak()) {
            RobotContainer.stateMachine.noteState = NoteState.Stored;
        }
            //Note is not stored
            else if (!io.getLoadedBeamBreak() && RobotContainer.stateMachine.shooterState != ShooterState.Shooting && !io.getEntryBeamBreak()) {
                RobotContainer.stateMachine.noteState = NoteState.Empty
        }
        //Note is intaking
        else if (io.getEntryBeamBreak() && !io.getLoadedBeamBreak()) {
            RobotContainer.stateMachine.noteState = NoteState.Intaking;
        }



        //Note is shot delay handling
        if (exitBreakBeamTriggerTime > 0.0) {
            if (Timer.getFPGATimestamp() - exitBreakBeamTriggerTime >= CannonConstants.NOTE_EXIT_BEAMBREAK_DELAY) {
                RobotContainer.stateMachine.noteState = NoteState.Empty;
                exitBreakBeamTriggerTime = -1.0;
            }
        }

        /*-----------------
            Shooter
           ---------------- */
        if (desiredLeftVel != RobotContainer.stateMachine.shooterState.leftVel || desiredRightVel != RobotContainer.stateMachine.shooterState.rightVel) {
            desiredLeftVel = RobotContainer.stateMachine.shooterState.leftVel
            desiredRightVel = RobotContainer.stateMachine.shooterState.rightVel

            io.setRightShooter(desiredRightVel)
            io.setLeftShooter(desiredLeftVel)

            println("set the shooters")
        }

        /*----------------
            Intake
         ------------------*/
        if (desiredInnerPercent != RobotContainer.stateMachine.intakeState.innerPercent || desiredOuterPercent != RobotContainer.stateMachine.intakeState.outerPercent) {
            desiredInnerPercent = RobotContainer.stateMachine.intakeState.innerPercent
            desiredOuterPercent = RobotContainer.stateMachine.intakeState.outerPercent

            io.setInnerIntakePercent(desiredInnerPercent)
            io.setOuterIntakePercent(desiredOuterPercent)
        }
    }

    override fun simulationPeriodic() {

    }
}
