package frc.robot.subsystems.trunk

import cshcyberhawks.lib.requests.IfRequest
import cshcyberhawks.lib.requests.ParallelRequest
import cshcyberhawks.lib.requests.Request
import cshcyberhawks.lib.requests.SequentialRequest
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.constants.TrunkConstants

class TrunkSystem() : SubsystemBase() {
    private val elevator = Elevator(ElevatorIOSim())
    private val pivot = Pivot(PivotIOSim())

    fun intakeRequest() =
        SequentialRequest(
            pivot.travelRequest(),
            ParallelRequest(
                elevator.intakeRequest(),
                pivot.coastRequest().withPrerequisite(elevator.positionBelow(0.36))
            )
        )

    fun stowRequest() = SequentialRequest(
        elevator.stowRequest(),
        IfRequest(
            elevator.positionBelow(0.2).booleanSupplier,
            SequentialRequest(
                pivot.brakeRequest().withPrerequisite(elevator.positionAbove(TrunkConstants.LEGAL_PIVOT_POSITION)),
                pivot.travelRequest()
            ),
        ),
        pivot.stowRequest()
    )
}