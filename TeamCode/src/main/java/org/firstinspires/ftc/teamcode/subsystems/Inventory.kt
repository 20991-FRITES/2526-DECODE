package org.firstinspires.ftc.teamcode.subsystems

import org.firstinspires.ftc.teamcode.BotContext
import org.firstinspires.ftc.teamcode.hardware.RobotHardware
import org.firstinspires.ftc.teamcode.interfaces.Subsystem
import org.firstinspires.ftc.teamcode.workers.InventorySnapshot
import org.firstinspires.ftc.teamcode.workers.InventoryWorker
import java.util.concurrent.Executors
import java.util.concurrent.Future

class Inventory(private val hardware: RobotHardware) : Subsystem {

    var artifacts = 0

    private val executor = Executors.newSingleThreadExecutor()
    private var future: Future<InventorySnapshot>? = null
    private var lastSnapshot: InventorySnapshot? = null

    private val DISTANCE_THRESHOLD = 10.0 // cm

    override fun periodic(context: BotContext) {
        // If worker finished, get result
        if (future?.isDone == true) {
            lastSnapshot = future?.get()
            future = null
        }

        // Start new worker if none running
        if (future == null) {
            future = executor.submit(InventoryWorker(hardware))
        }

        // Use latest snapshot
        lastSnapshot?.let { snapshot ->
            val distances = listOf(
                snapshot.distanceLeft,
                snapshot.distanceRight,
                snapshot.distanceIntakeLeft,
                snapshot.distanceIntakeRight
            )

            for (distance in distances) {
                if (distance < DISTANCE_THRESHOLD) {
                    artifacts++
                }
            }
        }
    }
}