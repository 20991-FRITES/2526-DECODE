package org.firstinspires.ftc.teamcode.workers

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.teamcode.hardware.RobotHardware
import java.util.concurrent.Callable

data class InventorySnapshot(
    val distanceLeft: Double,
    val distanceRight: Double,
    val distanceIntakeLeft: Double,
    val distanceIntakeRight: Double
)

class InventoryWorker(val hardware: RobotHardware) : Callable<InventorySnapshot> {
    override fun call():InventorySnapshot {
        val distanceLeft = hardware.distanceLeft.getDistance(DistanceUnit.CM)
        val distanceRight = hardware.distanceRight.getDistance(DistanceUnit.CM)
        val distanceIntakeLeft = hardware.distanceIntakeLeft.getDistance(DistanceUnit.CM)
        val distanceIntakeRight = hardware.distanceIntakeRight.getDistance(DistanceUnit.CM)

        return InventorySnapshot(
            distanceLeft,
            distanceRight,
            distanceIntakeLeft,
            distanceIntakeRight
        )
    }
}