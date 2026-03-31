package org.firstinspires.ftc.teamcode

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D
import org.firstinspires.ftc.teamcode.enums.Team
import org.firstinspires.ftc.teamcode.subsystems.Flywheel

data class BotContext(
    var botPose: Pose2D? = null,
    val team: Team,
    var flywheel: Flywheel? = null,
    var gamepad: com.qualcomm.robotcore.hardware.Gamepad?
)
