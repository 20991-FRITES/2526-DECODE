package org.firstinspires.ftc.teamcode.opmodes

import com.bylazar.telemetry.JoinedTelemetry
import com.bylazar.telemetry.PanelsTelemetry
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.seattlesolvers.solverslib.controller.PIDController
import com.seattlesolvers.solverslib.hardware.motors.MotorEx

@TeleOp(name = "Auto PID Velocity Tuner")
class PidTunerOpMode : OpMode() {

    lateinit var motor: MotorEx
    val joinedTelemetry = JoinedTelemetry(PanelsTelemetry.ftcTelemetry, telemetry)

    enum class TuneState {
        FIND_KU,
        COMPUTE_PID,
        RUN_PID
    }

    var state = TuneState.FIND_KU

    var kP = 0.0
    var kI = 0.0
    var kD = 0.0

    var Ku = 0.0
    var Pu = 0.0

    var targetVelocity = 1300.0

    var lastError = 0.0
    var lastCrossTime = 0L
    var crossings = 0

    // Rate-limit kP increments to once every 200ms instead of every loop tick
    val kpStepIntervalMs = 200L
    var lastKpStepTime = 0L
    var kpStep = 0.0003

    // Track only positive-going crossings for full-period measurement
    var lastPosCrossTime = 0L

    // Freeze kP once oscillation window begins
    var frozenKp = 0.0
    var kpFrozen = false

    private val pidController = PIDController(0.0, 0.0, 0.0)

    override fun init() {
        motor = MotorEx(hardwareMap, "CHANGE_ME")
    }

    override fun loop() {

        val velocity = motor.velocity
        val error = targetVelocity - velocity

        // Detect zero crossings
        if ((error > 0 && lastError < 0) || (error < 0 && lastError > 0)) {
            val now = System.currentTimeMillis()

            // Only measure full periods: positive-going crossings (negative→positive)
            if (error > 0 && lastError < 0) {
                if (lastPosCrossTime != 0L) {
                    Pu = (now - lastPosCrossTime) / 1000.0
                }
                lastPosCrossTime = now
            }

            crossings++

            // Freeze kP at the start of the oscillation window
            if (crossings == 1 && !kpFrozen) {
                frozenKp = kP
                kpFrozen = true
            }
        }

        lastError = error

        when (state) {

            TuneState.FIND_KU -> {
                kI = 0.0
                kD = 0.0

                if (!kpFrozen) {
                    // Rate-limited increment: only step kP every kpStepIntervalMs
                    val now = System.currentTimeMillis()
                    if (now - lastKpStepTime >= kpStepIntervalMs) {
                        kP += kpStep
                        lastKpStepTime = now
                    }
                } else {
                    kP = frozenKp
                }

                if (crossings > 8) {
                    Ku = frozenKp
                    state = TuneState.COMPUTE_PID
                }
            }

            TuneState.COMPUTE_PID -> {
                kP = 0.6 * Ku
                kI = 1.2 * Ku / Pu
                kD = 0.075 * Ku * Pu
                pidController.reset()
                state = TuneState.RUN_PID
            }

            TuneState.RUN_PID -> {
                // Normal PID operation
            }
        }

        pidController.setPID(kP, kI, kD)
        val power = pidController.calculate(velocity, targetVelocity)

        motor.set(power.coerceIn(-1.0, 1.0))

        joinedTelemetry.addData("State", state)
        joinedTelemetry.addData("Velocity", velocity)
        joinedTelemetry.addData("Target", targetVelocity)
        joinedTelemetry.addData("Error", error)
        joinedTelemetry.addData("Power", power)
        joinedTelemetry.addData("kP", kP)
        joinedTelemetry.addData("kI", kI)
        joinedTelemetry.addData("kD", kD)
        joinedTelemetry.addData("Ku", Ku)
        joinedTelemetry.addData("Pu", Pu)
        joinedTelemetry.addData("Crossings", crossings)
        joinedTelemetry.update()
    }
}