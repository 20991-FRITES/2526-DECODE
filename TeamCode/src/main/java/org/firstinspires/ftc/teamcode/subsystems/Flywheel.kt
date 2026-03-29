package org.firstinspires.ftc.teamcode.subsystems

import com.seattlesolvers.solverslib.controller.PIDFController
import org.firstinspires.ftc.teamcode.enums.FlywheelState
import org.firstinspires.ftc.teamcode.hardware.RobotHardware
import org.firstinspires.ftc.teamcode.interfaces.Subsystem

class Flywheel(private val hardware: RobotHardware) : Subsystem {
    private var state: FlywheelState = FlywheelState.ON

    private val pidfController: PIDFController = PIDFController(0.1, 0.01, 0.05, 0.0);

    private val targetVelocity: Double = 1300.0 // ticks/second

    override fun periodic() {
        when (state) {
            FlywheelState.ON -> {
                val currentVelocityLeft = hardware.flywheelLeft.velocity
                val currentVelocityRight = hardware.flywheelRight.velocity
                val powerLeft = pidfController.calculate(currentVelocityLeft, targetVelocity)
                val powerRight = pidfController.calculate(currentVelocityRight, targetVelocity)
                hardware.flywheelLeft.set(powerLeft)
                hardware.flywheelRight.set(powerRight)
            }
            FlywheelState.OFF -> {
                hardware.flywheelLeft.set(0.0)
                hardware.flywheelRight.set(0.0)
            }
        }
    }
}