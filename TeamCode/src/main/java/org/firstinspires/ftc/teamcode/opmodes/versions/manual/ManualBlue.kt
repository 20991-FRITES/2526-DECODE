package org.firstinspires.ftc.teamcode.opmodes.versions.manual

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.enums.Team
import org.firstinspires.ftc.teamcode.opmodes.ManualOpMode

@TeleOp(name = "Manual BLUE", group = "Manual")
class ManualBlue : ManualOpMode() {
    init {
        team = Team.BLUE
    }
}