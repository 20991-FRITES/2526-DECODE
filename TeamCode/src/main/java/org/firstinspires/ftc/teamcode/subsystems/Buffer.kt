package org.firstinspires.ftc.teamcode.subsystems

import org.firstinspires.ftc.teamcode.enums.BufferState
import org.firstinspires.ftc.teamcode.hardware.RobotHardware
import org.firstinspires.ftc.teamcode.interfaces.Subsystem

class Buffer(private val hardware: RobotHardware) : Subsystem {
    var state: BufferState = BufferState.OFF

    private var minShotTime: Double =
        500.0 // ms - Minimum time that the buffer must be on before moving to the next shot.

    private var lastShotTime: Long =
        0 // Timestamp of the last time the buffer was turned on for shooting.


    fun changeState(newState: BufferState) {
        if (newState == BufferState.WAITING_LEFT || newState == BufferState.WAITING_RIGHT) {
            lastShotTime = System.currentTimeMillis()

            state = newState
        }
    }

    override fun periodic(flywheel: Flywheel) {
        when (state) {
            BufferState.OFF -> {
                hardware.bufferLeft.set(0.0)
                hardware.bufferRight.set(0.0)
            }

            BufferState.LEFT_SHOOT -> {
                hardware.bufferLeft.set(1.0)
                hardware.bufferRight.set(0.0)

                if (System.currentTimeMillis() - lastShotTime >= minShotTime) {
                    state = BufferState.WAITING_RIGHT
                }
            }

            BufferState.RIGHT_SHOOT -> {
                hardware.bufferLeft.set(0.0)
                hardware.bufferRight.set(1.0)

                if (System.currentTimeMillis() - lastShotTime >= minShotTime) {
                    state = BufferState.WAITING_LEFT
                }
            }

            BufferState.WAITING_LEFT -> {
                hardware.bufferLeft.set(0.0)
                hardware.bufferRight.set(0.0)

                if (!flywheel.isAtTargetVelocity()) return

                changeState(BufferState.LEFT_SHOOT)
            }

            BufferState.WAITING_RIGHT -> {
                hardware.bufferLeft.set(0.0)
                hardware.bufferRight.set(0.0)

                if (!flywheel.isAtTargetVelocity()) return

                changeState(BufferState.RIGHT_SHOOT)
            }

            BufferState.REVERSE -> {
                hardware.bufferLeft.set(-1.0)
                hardware.bufferRight.set(-1.0)
            }
        }
    }
}