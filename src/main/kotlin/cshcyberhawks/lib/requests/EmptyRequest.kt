package cshcyberhawks.lib.requests

class EmptyRequest() : Request() {
    override fun execute() {}

    override fun isFinished(): Boolean = true
}