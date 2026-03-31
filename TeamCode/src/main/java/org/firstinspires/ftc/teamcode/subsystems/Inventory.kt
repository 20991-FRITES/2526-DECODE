package org.firstinspires.ftc.teamcode.subsystems

import org.firstinspires.ftc.teamcode.BotContext
import org.firstinspires.ftc.teamcode.enums.Team
import org.firstinspires.ftc.teamcode.hardware.RobotHardware
import org.firstinspires.ftc.teamcode.interfaces.Subsystem

class Inventory(private val hardware: RobotHardware) : Subsystem {
    private var artifacts = 0

    override fun periodic(context: BotContext) {
    }
}