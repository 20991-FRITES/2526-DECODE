package org.firstinspires.ftc.teamcode.opmodes

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.seattlesolvers.solverslib.gamepad.GamepadKeys
import org.firstinspires.ftc.teamcode.enums.BufferState
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

        intake.setState(
            when {
                intakeForward -> IntakeState.ON
                intakeReverse -> IntakeState.REVERSE
                else -> IntakeState.OFF
            }
        )

        buffer.setState(
            when {
                shoot -> BufferState.WAITING_LEFT
                intakeReverse -> BufferState.REVERSE
                shootRelease -> BufferState.OFF
                else -> buffer.state
            }
        )

        localization.periodic()
        drivetrain.periodic()
        intake.periodic()
        flywheel.periodic()
        buffer.periodic()
    }
}