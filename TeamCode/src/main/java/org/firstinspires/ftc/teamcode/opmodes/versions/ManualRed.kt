package org.firstinspires.ftc.teamcode.opmodes.versions

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.enums.Team
import org.firstinspires.ftc.teamcode.opmodes.ManualOpMode

@TeleOp(name = "Manual RED", group = "Manual")
class ManualRed : ManualOpMode() {
    init {
        team = Team.RED
    }
}