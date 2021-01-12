package org.firstinspires.ftc.teamcode.Team9113.Autonomous


fun delay(delay: Long, block: () -> Unit) {
    GlobalScope.launch {
        Thread.sleep(delay)
        block()
    }
}