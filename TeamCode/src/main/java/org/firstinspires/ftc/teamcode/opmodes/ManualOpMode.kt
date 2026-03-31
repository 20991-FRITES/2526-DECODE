package org.firstinspires.ftc.teamcode.opmodes

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.seattlesolvers.solverslib.gamepad.GamepadKeys
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
import org.firstinspires.ftc.teamcode.subsystems.Localization

open class ManualOpMode : OpMode() {
    var robotHardware: RobotHardware = RobotHardware()
    lateinit var drivetrain: Drivetrain
    lateinit var localization: Localization
    lateinit var intake: Intake
    lateinit var flywheel: Flywheel
    lateinit var buffer: Buffer
    lateinit var team: Team

    lateinit var botContext: BotContext

    lateinit var subsystems: List<Subsystem>
    private var rightTriggerPrev = false

    override fun init() {
        robotHardware.init(hardwareMap, gamepad1)

        localization = Localization(robotHardware)
        drivetrain = Drivetrain(robotHardware, team)
        intake = Intake(robotHardware)
        flywheel = Flywheel(robotHardware)
        buffer = Buffer(robotHardware)

        botContext = BotContext(
            team = team,
        )

        subsystems = listOf(
            localization,
            drivetrain,
            intake,
            flywheel,
            buffer
        )
    }

    override fun loop() {
        val intakeForward = robotHardware.gamepad.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.5
        val intakeReverse = robotHardware.gamepad.isDown(GamepadKeys.Button.DPAD_DOWN)
        val rightTriggerNow = robotHardware.gamepad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.5
        val shoot = rightTriggerNow && !rightTriggerPrev
        val shootRelease = !rightTriggerNow && rightTriggerPrev
        val flywheelToggle = robotHardware.gamepad.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)
        val lockTowardsGoal = robotHardware.gamepad.isDown(GamepadKeys.Button.LEFT_BUMPER)

        val touchedJoystick =
            robotHardware.gamepad.leftX >= 0.1 || robotHardware.gamepad.leftX <= -0.1 || robotHardware.gamepad.leftY >= 0.1 || robotHardware.gamepad.leftY <= -0.1
        val macroMoveToShoot = robotHardware.gamepad.wasJustPressed(GamepadKeys.Button.X)

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

        robotHardware.gamepad.readButtons()

        rightTriggerPrev = rightTriggerNow
    }
}