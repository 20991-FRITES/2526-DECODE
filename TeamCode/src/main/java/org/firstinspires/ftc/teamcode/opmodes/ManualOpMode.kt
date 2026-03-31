package org.firstinspires.ftc.teamcode.opmodes

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import org.firstinspires.ftc.teamcode.BotContext
import org.firstinspires.ftc.teamcode.enums.BufferState
import org.firstinspires.ftc.teamcode.enums.DrivetrainState
import org.firstinspires.ftc.teamcode.enums.IntakeState
import org.firstinspires.ftc.teamcode.enums.Team
import org.firstinspires.ftc.teamcode.hardware.RobotHardware
import org.firstinspires.ftc.teamcode.interfaces.Subsystem
import org.firstinspires.ftc.teamcode.subsystems.Buffer
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain
import org.firstinspires.ftc.teamcode.subsystems.Flywheel
import org.firstinspires.ftc.teamcode.subsystems.Intake
import org.firstinspires.ftc.teamcode.subsystems.Inventory
import org.firstinspires.ftc.teamcode.subsystems.Localization

open class ManualOpMode : OpMode() {
    var robotHardware: RobotHardware = RobotHardware()
    lateinit var drivetrain: Drivetrain
    lateinit var localization: Localization
    lateinit var intake: Intake
    lateinit var flywheel: Flywheel
    lateinit var buffer: Buffer
    lateinit var team: Team
    lateinit var inventory: Inventory

    lateinit var botContext: BotContext

    lateinit var subsystems: List<Subsystem>
    private var rightTriggerPrev = false
    private var rumbledForThreeArtifacts = false

    override fun init() {
        robotHardware.init(hardwareMap, gamepad1)

        localization = Localization(robotHardware)
        drivetrain = Drivetrain(robotHardware, team)
        intake = Intake(robotHardware)
        flywheel = Flywheel(robotHardware)
        buffer = Buffer(robotHardware)
        inventory = Inventory(robotHardware)

        botContext = BotContext(
            team = team,
            gamepad = gamepad1,
        )

        subsystems = listOf(
            localization,
            drivetrain,
            intake,
            flywheel,
            buffer,
            inventory
        )
    }

    override fun loop() {
        val intakeForward = gamepad1.left_trigger_pressed
        val intakeReverse = gamepad1.dpad_down
        val rightTriggerNow = gamepad1.right_trigger_pressed
        val shoot = rightTriggerNow && !rightTriggerPrev
        val shootRelease = !rightTriggerNow && rightTriggerPrev
        val flywheelToggle = gamepad1.dpadLeftWasPressed()
        val lockTowardsGoal = gamepad1.left_bumper

        val touchedJoystick =
            gamepad1.left_stick_x != 0.0f || gamepad1.left_stick_y != 0.0f || gamepad1.right_stick_x != 0.0f
        val macroMoveToShoot = gamepad1.xWasPressed()

        intake.setState(
            when {
                intakeForward -> IntakeState.ON
                intakeReverse -> IntakeState.REVERSE
                else -> IntakeState.OFF
            }
        )

        buffer.changeState(
            when {
                shoot -> BufferState.WAITING_LEFT
                intakeReverse -> BufferState.REVERSE
                shootRelease -> BufferState.OFF
                else -> buffer.state
            }
        )

        if (flywheelToggle) flywheel.toggleState()


        drivetrain.changeState(
            when {
                lockTowardsGoal -> DrivetrainState.LOCK_TOWARDS_GOAL
                macroMoveToShoot -> DrivetrainState.MACRO_MOVE_TO_SHOOT
                touchedJoystick -> DrivetrainState.DRIVER_CONTROLLED_ROBOT_CENTRIC
                else -> DrivetrainState.DRIVER_CONTROLLED_FIELD_CENTRIC
            }
        )

        for (subsystem in subsystems) {
            subsystem.periodic(botContext)
        }


        if (inventory.artifacts >= 3 && !rumbledForThreeArtifacts) {
            gamepad1.rumble(1000)
            rumbledForThreeArtifacts = true
        }

        if (inventory.artifacts < 3) {
            rumbledForThreeArtifacts = false
        }

        rightTriggerPrev = rightTriggerNow
    }
}