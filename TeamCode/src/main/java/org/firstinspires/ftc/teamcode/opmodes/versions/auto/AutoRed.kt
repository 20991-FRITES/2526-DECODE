package org.firstinspires.ftc.teamcode.opmodes.versions.auto

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.enums.Team
import org.firstinspires.ftc.teamcode.opmodes.AutoOpMode

@Autonomous(name = "Auto Red", group = "Auto")
class AutoRed: AutoOpMode() {
    init {
        team = Team.RED
    }
}