package cshcyberhawks.lib.requests

abstract class Prerequisite {
    companion object {
        fun withCondition(condition: () -> Boolean) = object : Prerequisite() {
            override fun met(): Boolean = condition()
        }
    }

    abstract fun met(): Boolean

    val booleanSupplier: () -> Boolean
        get() =
            { met() }
}