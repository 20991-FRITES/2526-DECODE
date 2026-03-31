package org.firstinspires.ftc.teamcode.subsystems

import com.seattlesolvers.solverslib.controller.PIDFController
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D
import org.firstinspires.ftc.teamcode.BotContext
import org.firstinspires.ftc.teamcode.config.FieldConfig
import org.firstinspires.ftc.teamcode.config.FieldConfig.mirror
import org.firstinspires.ftc.teamcode.enums.FlywheelState
import org.firstinspires.ftc.teamcode.enums.Team
import org.firstinspires.ftc.teamcode.hardware.RobotHardware
import org.firstinspires.ftc.teamcode.interfaces.Subsystem
import kotlin.math.abs
import kotlin.math.hypot
import kotlin.math.pow
import kotlin.math.sqrt

class Flywheel(private val hardware: RobotHardware) : Subsystem {
    private var state: FlywheelState = FlywheelState.ON

    private val pidfController: PIDFController = PIDFController(0.02, 0.0, 0.0, -0.5);

    private var targetVelocity: Double = 1300.0 // ticks/second

    fun distanceToVelocity(distance: Double, distanceUnit: DistanceUnit): Double {
        val d = distanceUnit.toCm(distance)
        return (185 * d * d) / 86658 + (26035 * d) / 86658 + (1209050 / 1111).toDouble() // TODO: Calibrate this
    }

    fun toggleState() {
        state = when (state) {
            FlywheelState.ON -> FlywheelState.OFF
            FlywheelState.OFF -> FlywheelState.ON
        }
    }

    override fun periodic(context: BotContext) {
        when (state) {
            FlywheelState.ON -> {
                val target = mirror(FieldConfig.redGoalPose, context.team)
                val distanceToTargetCM = hypot(
                    x = target.getX(DistanceUnit.CM) - context.botPose!!.getX(DistanceUnit.CM),
                    y = target.getY(DistanceUnit.CM) - context.botPose!!.getY(DistanceUnit.CM)
                )

                targetVelocity = distanceToVelocity(distanceToTargetCM, DistanceUnit.CM)

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

        context.flywheel = this
    }

    fun isAtTargetVelocity(): Boolean {
        val currentVelocityLeft = hardware.flywheelLeft.velocity
        val currentVelocityRight = hardware.flywheelRight.velocity
        val velocityTolerance = 50.0 // ticks/second
        return abs(currentVelocityLeft - targetVelocity) < velocityTolerance &&
                abs(currentVelocityRight - targetVelocity) < velocityTolerance
    }
}