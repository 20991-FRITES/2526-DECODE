package org.firstinspires.ftc.teamcode.subsystems

import org.firstinspires.ftc.teamcode.BotContext
import org.firstinspires.ftc.teamcode.enums.IntakeState
import org.firstinspires.ftc.teamcode.hardware.RobotHardware
import org.firstinspires.ftc.teamcode.interfaces.Subsystem

class Intake(private val hardware: RobotHardware) : Subsystem {
    private var state: IntakeState = IntakeState.OFF

    fun setState(newState: IntakeState) {
        state = newState
    }

    override fun periodic(context: BotContext) {
        when (state) {
            IntakeState.ON -> hardware.intake.set(1.0)
            IntakeState.OFF -> hardware.intake.set(0.0)
            IntakeState.REVERSE -> hardware.intake.set(-1.0)
        }
    }
}