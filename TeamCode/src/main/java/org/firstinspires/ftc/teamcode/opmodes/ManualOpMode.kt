package org.firstinspires.ftc.teamcode.opmodes

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.seattlesolvers.solverslib.gamepad.GamepadKeys
import org.firstinspires.ftc.teamcode.enums.BufferState
import org.firstinspires.ftc.teamcode.enums.DrivetrainState
import org.firstinspires.ftc.teamcode.enums.IntakeState
import org.firstinspires.ftc.teamcode.hardware.RobotHardware
import org.firstinspires.ftc.teamcode.subsystems.Buffer
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain
import org.firstinspires.ftc.teamcode.subsystems.Flywheel
import org.firstinspires.ftc.teamcode.subsystems.Intake
import org.firstinspires.ftc.teamcode.subsystems.Localization

class ManualOpMode : OpMode() {
    var robotHardware: RobotHardware = RobotHardware()
    lateinit var drivetrain: Drivetrain
    lateinit var localization: Localization
    lateinit var intake: Intake
    lateinit var flywheel: Flywheel
    lateinit var buffer: Buffer

    override fun init() {
        robotHardware.init(hardwareMap, gamepad1)

        localization = Localization(robotHardware)
        drivetrain = Drivetrain(robotHardware)
        intake = Intake(robotHardware)
        flywheel = Flywheel(robotHardware)
        buffer = Buffer(robotHardware)
    }

    override fun loop() {
        val intakeForward = robotHardware.gamepad.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.5
        val intakeReverse = robotHardware.gamepad.isDown(GamepadKeys.Button.DPAD_DOWN)
        val shoot = robotHardware.gamepad.wasJustPressed(GamepadKeys.Trigger.RIGHT_TRIGGER)
        val shootRelease = robotHardware.gamepad.wasJustReleased(GamepadKeys.Trigger.RIGHT_TRIGGER)
        val flywheelToggle = robotHardware.gamepad.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)
        val lockTowardsGoal = robotHardware.gamepad.isDown(GamepadKeys.Button.LEFT_BUMPER)

        val touchedJoystick = robotHardware.gamepad.leftX >= 0.1 || robotHardware.gamepad.leftX <= -0.1 || robotHardware.gamepad.leftY >= 0.1 || robotHardware.gamepad.leftY <= -0.1
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

        localization.periodic()

        val botPose = localization.getPose()

        drivetrain.changeState(
            when {
                lockTowardsGoal -> DrivetrainState.LOCK_TOWARDS_GOAL
                macroMoveToShoot -> DrivetrainState.MACRO_MOVE_TO_SHOOT
                touchedJoystick -> DrivetrainState.DRIVER_CONTROLLED_ROBOT_CENTRIC
                else -> DrivetrainState.DRIVER_CONTROLLED_FIELD_CENTRIC
            }
        )

        drivetrain.periodic(botPose)
        flywheel.periodic(botPose)
        intake.periodic()
        buffer.periodic()
    }
}