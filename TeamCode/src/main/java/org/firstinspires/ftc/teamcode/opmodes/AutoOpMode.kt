package org.firstinspires.ftc.teamcode.opmodes

import com.pedropathing.follower.Follower
import com.pedropathing.ftc.InvertedFTCCoordinates
import com.pedropathing.ftc.PoseConverter
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import org.firstinspires.ftc.teamcode.config.AutoConfig.END_TIME_BUFFER
import org.firstinspires.ftc.teamcode.config.AutoConfig.INTAKE_FROM_RAMP_DURATION
import org.firstinspires.ftc.teamcode.config.AutoPaths
import org.firstinspires.ftc.teamcode.config.FieldConfig.mirrorPedro
import org.firstinspires.ftc.teamcode.enums.AutoState
import org.firstinspires.ftc.teamcode.enums.BufferState
import org.firstinspires.ftc.teamcode.enums.IntakeState
import org.firstinspires.ftc.teamcode.enums.Team
import org.firstinspires.ftc.teamcode.hardware.RobotHardware
import org.firstinspires.ftc.teamcode.pedropathing.Constants
import org.firstinspires.ftc.teamcode.subsystems.Buffer
import org.firstinspires.ftc.teamcode.subsystems.Flywheel
import org.firstinspires.ftc.teamcode.subsystems.Intake

open class AutoOpMode : OpMode() {
    lateinit var follower: Follower;
    lateinit var team: Team

    var state = AutoState.MOVE_TO_SHOOT

    var lastPathEndPose = AutoPaths.startPose

    lateinit var intake: Intake
    lateinit var flywheel: Flywheel
    lateinit var buffer: Buffer
    var robotHardware: RobotHardware = RobotHardware()
    var nbShots: Int = 0 // Number of shooting cycles
    var intakeStartTime = 0L // Timestamp when the intake was turned on

    var auto_start_time = 0L // Timestamp when the autonomous period starts

    override fun init() {
        robotHardware.init(hardwareMap, gamepad1)

        intake = Intake(robotHardware)
        flywheel = Flywheel(robotHardware)
        buffer = Buffer(robotHardware)

        follower = Constants.createFollower(hardwareMap)

        follower.setStartingPose(
            mirrorPedro(AutoPaths.startPose, team),
        )
    }

    override fun start() {
        auto_start_time = System.currentTimeMillis()
    }

    override fun loop() {
        follower.update()

        intake.setState(IntakeState.ON)

        flywheel.periodic(
            PoseConverter.poseToPose2D(
                follower.pose,
                InvertedFTCCoordinates.INSTANCE
            )
        )
        intake.periodic()
        buffer.periodic()

        if (System.currentTimeMillis() - auto_start_time >= 30_000 - END_TIME_BUFFER) {
            // If there's END_TIME_BUFFER or less left in the autonomous period, stop all actions and park.
            state = AutoState.PARK
        }
        autonomousPathUpdate()
    }

    fun autonomousPathUpdate() {
        when (state) {
            AutoState.MOVE_TO_SHOOT -> {
                if (!follower.isBusy) {
                    val path = AutoPaths.buildMoveToShootPath(lastPathEndPose, follower)

                    follower.followPath(path)

                    lastPathEndPose = path.endPose()

                    state = AutoState.SHOOT
                }
            }

            AutoState.SHOOT -> {
                buffer.changeState(BufferState.WAITING_LEFT)
                state = AutoState.SHOOTING
            }

            AutoState.SHOOTING -> {
                if (buffer.shotCount >= 3) {
                    nbShots++
                    if (nbShots == 1) state = AutoState.MOVE_TO_INTAKE_FROM_MIDDLE_ROW
                    else if (nbShots == 2 || nbShots == 3) state =
                        AutoState.MOVE_TO_INTAKE_FROM_RAMP
                    else if (nbShots == 4) state = AutoState.MOVE_TO_INTAKE_FROM_NORTH_ROW
                    else state = AutoState.PARK
                }
            }

            AutoState.MOVE_TO_INTAKE_FROM_RAMP -> {
                if (!follower.isBusy) {
                    val path = AutoPaths.buildIntakeFromRampPath(lastPathEndPose, follower)

                    follower.followPath(path)

                    lastPathEndPose = path.endPose()

                    state = AutoState.INTAKE_FROM_RAMP
                }
            }

            AutoState.INTAKE_FROM_RAMP -> {
                if (!follower.isBusy) {
                    intake.setState(IntakeState.ON)

                    if (intakeStartTime == 0L) {
                        intakeStartTime = System.currentTimeMillis()
                    }

                    if (System.currentTimeMillis() - intakeStartTime >= INTAKE_FROM_RAMP_DURATION) { // Run intake for 3 seconds
                        intake.setState(IntakeState.OFF)
                        intakeStartTime = 0L
                        state = AutoState.MOVE_TO_SHOOT
                    }
                }
            }

            AutoState.MOVE_TO_INTAKE_FROM_NORTH_ROW -> {
                if (!follower.isBusy) {
                    val path = AutoPaths.buildMoveToNorthRowPath(lastPathEndPose, follower)

                    follower.followPath(path)

                    lastPathEndPose = path.endPose()

                    state = AutoState.INTAKE_FORWARDS
                }
            }

            AutoState.MOVE_TO_INTAKE_FROM_MIDDLE_ROW -> {
                if (!follower.isBusy) {
                    val path = AutoPaths.buildMoveToMiddleRowPath(lastPathEndPose, follower)

                    follower.followPath(path)

                    lastPathEndPose = path.endPose()

                    state = AutoState.INTAKE_FORWARDS
                }
            }

            AutoState.INTAKE_FORWARDS -> {
                if (!follower.isBusy) {
                    intake.setState(IntakeState.ON)

                    val path = AutoPaths.buildIntakeStraightPath(lastPathEndPose, follower)

                    follower.followPath(path)

                    lastPathEndPose = path.endPose()

                    state = AutoState.MOVE_TO_SHOOT
                }
            }

            AutoState.PARK -> {
                if (!follower.isBusy) {
                    val path = AutoPaths.buildMoveToParkPath(lastPathEndPose, follower)

                    follower.followPath(path)

                    lastPathEndPose = path.endPose()

                    state = AutoState.DONE
                }
            }

            AutoState.DONE -> {
                // Do nothing, we're parked and the autonomous period is almost over.
            }
        }
    }
}