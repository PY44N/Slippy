package frc.robot.subsystems.cannon

import MiscCalculations
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.*
import frc.robot.constants.CannonConstants

class CannonSystem(val io: CannonIO) : SubsystemBase() {

    //Desired shooter velocities
    var desiredLeftVel = 0.0
    var desiredRightVel = 0.0

    //Desired intake percentages
    var desiredOuterPercent = 0.0
    var desiredInnerPercent = 0.0


    private var exitBreakBeamTriggerTime: Double = -1.0;


    init {

    }


    fun killShooter() {
        RobotContainer.stateMachine.shooterState = ShooterState.Stopped
    }

    fun prep() {
        RobotContainer.stateMachine.shooterState = ShooterState.Prepped;
    }

    fun shoot() {
        RobotContainer.stateMachine.shooterState = ShooterState.Shooting;
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
        if (MiscCalculations.appxEqual(RobotContainer.stateMachine.shooterState.leftVel, io.getLeftShooterVel(), CannonConstants.SHOOTER_VELOCITY_DEADZONE) && MiscCalculations.appxEqual(RobotContainer.stateMachine.shooterState.rightVel, io.getRightShooterVel(), CannonConstants.SHOOTER_VELOCITY_DEADZONE)) {
            return true
        }
        else {
            return false
        }
    }

    override fun periodic() {
        /*--------------------
             Beam Breaks
        -----------------------*/

        //Note is shot
        if (io.getExitBeamBreak()) {
            exitBreakBeamTriggerTime = Timer.getFPGATimestamp()
        }
        //Note is stored
        else if (io.getLoadedBeamBreak()) {
            RobotContainer.stateMachine.noteState = NoteState.Stored;
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

            io.setLeftShooterVel(desiredLeftVel)
            io.setRightShooterVel(desiredRightVel)
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
