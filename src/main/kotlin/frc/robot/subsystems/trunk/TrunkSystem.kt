package frc.robot.subsystems.trunk

import cshcyberhawks.lib.requests.ParallelRequest
import cshcyberhawks.lib.requests.Request
import cshcyberhawks.lib.requests.SequentialRequest
import edu.wpi.first.wpilibj2.command.SubsystemBase

class TrunkSystem() : SubsystemBase() {
    private val elevator = Elevator(ElevatorIOSim())
    private val pivot = Pivot(PivotIOSim())

    fun intakeRequest() =
        SequentialRequest(
            pivot.travelRequest(),
            ParallelRequest(
                elevator.intakeRequest(),
                pivot.coastRequest().withPrerequisite(elevator.elevatorBelow(0.36))
            )
        )

    fun stowRequest() = SequentialRequest(

    )
}