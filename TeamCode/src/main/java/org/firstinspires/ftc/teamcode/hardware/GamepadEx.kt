package org.firstinspires.ftc.teamcode.hardware

import com.qualcomm.robotcore.hardware.Gamepad
import com.seattlesolvers.solverslib.gamepad.GamepadKeys

class GamepadEx(gamepad: Gamepad) :
    com.seattlesolvers.solverslib.gamepad.GamepadEx(gamepad) {

    private val lastTriggerStates = mutableMapOf<GamepadKeys.Trigger, Boolean>()

    fun wasJustPressed(trigger: GamepadKeys.Trigger, threshold: Double = 0.5): Boolean {
        val current = getTrigger(trigger) > threshold
        val last = lastTriggerStates[trigger] ?: false

        lastTriggerStates[trigger] = current

        return current && !last
    }

    fun wasJustReleased(trigger: GamepadKeys.Trigger, threshold: Double = 0.5): Boolean {
        val current = getTrigger(trigger) > threshold
        val last = lastTriggerStates[trigger] ?: false

        lastTriggerStates[trigger] = current

        return !current && last
    }
}