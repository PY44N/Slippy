package cshcyberhawks.lib.requests

class IfRequest(
    private val condition: () -> Boolean,
    private val ifBranch: Request,
    private val elseBranch: Request = EmptyRequest()
) : Request() {
    private var hasStarted = false
    private var activeBranch: Request = EmptyRequest()

    private fun startIfAllowed() {
        if (!hasStarted && activeBranch.allowed()) {
            activeBranch.execute()
            hasStarted = true
        }
    }

    override fun execute() {
        activeBranch = if (condition()) {
            ifBranch
        } else {
            elseBranch
        }

        startIfAllowed()
    }

    override fun isFinished(): Boolean {
        startIfAllowed()

        return hasStarted && activeBranch.isFinished()
    }
}