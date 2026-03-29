package org.firstinspires.ftc.teamcode.interfaces

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D
import org.firstinspires.ftc.teamcode.subsystems.Flywheel

interface Subsystem {
    fun periodic() {
    }

    fun periodic(botpose: Pose2D) {
    }

    fun periodic(flywheel: Flywheel) {
    }
}