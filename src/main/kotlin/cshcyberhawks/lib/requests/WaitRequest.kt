package cshcyberhawks.lib.requests

import cshcyberhawks.lib.math.Timer

class WaitRequest(private val time: Double) : Request() {
    val timer = Timer()

    override fun execute() {
        timer.reset()
        timer.start()
    }

    override fun isFinished(): Boolean {
        return timer.hasElapsed(time)
    }
}