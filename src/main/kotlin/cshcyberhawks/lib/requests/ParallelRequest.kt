package cshcyberhawks.lib.requests

class ParallelRequest(vararg requests: Request) : Request() {
    private val idleRequests = requests.toMutableList()
    private val inProgressRequests: MutableList<Request> = mutableListOf()

    private fun startRequests() {
        for (request in idleRequests) {
            if (request.allowed()) {
                request.execute()
                inProgressRequests.add(request)
                idleRequests.remove(request)
            }
        }
    }

    override fun execute() {
        startRequests()
    }

    override fun isFinished(): Boolean {
        startRequests()
        inProgressRequests.removeIf { request -> request.isFinished() }

        return idleRequests.isEmpty() && inProgressRequests.isEmpty()
    }
}