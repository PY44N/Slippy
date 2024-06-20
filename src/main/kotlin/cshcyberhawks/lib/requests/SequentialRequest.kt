package cshcyberhawks.lib.requests

import java.util.*

class SequentialRequest(vararg requests: Request) : Request() {
    private val idleRequests = requests.toMutableList()
    private var currentRequest = if (idleRequests.isNotEmpty()) idleRequests.removeAt(0) else EmptyRequest()
    private var startedCurrentRequest = false

    private fun startNextRequest() {
        if (idleRequests.isNotEmpty() && currentRequest.isFinished()) {
            currentRequest = idleRequests.removeAt(0)
            startedCurrentRequest = false
            startIfAllowed()
        }
    }

    private fun startIfAllowed() {
        if (!startedCurrentRequest && currentRequest.allowed()) {
            currentRequest.execute()
            startedCurrentRequest = true
        }
    }

    override fun execute() {
        startIfAllowed()
        startNextRequest()
    }

    override fun isFinished(): Boolean {
        startIfAllowed()
        startNextRequest()

        return idleRequests.isEmpty() && startedCurrentRequest && currentRequest.isFinished()
    }
}