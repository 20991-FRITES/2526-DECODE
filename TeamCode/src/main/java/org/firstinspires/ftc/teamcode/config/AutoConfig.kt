package org.firstinspires.ftc.teamcode.config

object AutoConfig {
    val INTAKE_FROM_RAMP_DURATION =
        1500L // Duration to run the intake when intaking from the ramp (in milliseconds)
    val END_TIME_BUFFER = 1000L // Time buffer to ensure the robot parks before the end of the autonomous period (in milliseconds)
    // basically, if the robot is still doing something when END_TIME_BUFFER milliseconds are left, it will stop and park immediately
}