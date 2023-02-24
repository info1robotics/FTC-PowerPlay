package org.firstinspires.ftc.teamcode.opmodes

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.acmerobotics.roadrunner.trajectory.Trajectory
import com.acmerobotics.roadrunner.trajectory.constraints.*
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.commands.*
import org.firstinspires.ftc.teamcode.moonshine.PrincipledTeleOpMode
import org.firstinspires.ftc.teamcode.moonshine.SharedSubsystem
import org.firstinspires.ftc.teamcode.moonshine.builtin.ContinuousReuseCommand
import org.firstinspires.ftc.teamcode.moonshine.builtin.ParallelCommand
import org.firstinspires.ftc.teamcode.moonshine.builtin.SerialCommand
import org.firstinspires.ftc.teamcode.moonshine.extensions.*
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence
import org.firstinspires.ftc.teamcode.subsystems.LinkageSubsystem
import org.firstinspires.ftc.teamcode.subsystems.MecanumSubsystem
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem
import java.util.*
import kotlin.run


@Autonomous
class AutoLeftHigh : PrincipledTeleOpMode() {
    private val linkageSubsystem = SharedSubsystem(LinkageSubsystem::class.java, LinkageSubsystem())
    private val turretSubsystem = SharedSubsystem(TurretSubsystem::class.java, TurretSubsystem())
    private val mecanumSubsystem = SharedSubsystem(MecanumSubsystem::class.java, MecanumSubsystem())

    lateinit var stack_to_high_1: TrajectorySequence
    lateinit var stack_to_high_2: TrajectorySequence
    lateinit var align_preload: Trajectory
    lateinit var high_align_1: TrajectorySequence
    lateinit var high_align_2: TrajectorySequence
    lateinit var preload_to_turn: TrajectorySequence
    lateinit var high_to_stack_1: TrajectorySequence
    lateinit var high_to_stack_2: TrajectorySequence
    lateinit var turn_to_stack: TrajectorySequence
    lateinit var go_to_stack: TrajectorySequence
    lateinit var low_to_stack_1: TrajectorySequence
    lateinit var low_to_stack_2: TrajectorySequence
    lateinit var low_to_stack_3: TrajectorySequence
    lateinit var low_to_stack_4: TrajectorySequence


    var slowConstraint: TrajectoryVelocityConstraint = MinVelocityConstraint(
        listOf(
            TranslationalVelocityConstraint(40.0),
            AngularVelocityConstraint(2.0)
        )
    )

    var fastConstraint: TrajectoryVelocityConstraint = MinVelocityConstraint(
        listOf(
            TranslationalVelocityConstraint(80.0)
        )
    )
    var accelConstraint: TrajectoryAccelerationConstraint = ProfileAccelerationConstraint(20.0)
    var fastAccelConstraint: TrajectoryAccelerationConstraint = ProfileAccelerationConstraint(80.0)
    override fun getSubsystems(): ContinuousReuseCommand = newContinuous {
        add(linkageSubsystem)
        add(turretSubsystem)
        add(mecanumSubsystem)
    }

    override fun getInitRoutine(): SerialCommand = newSerial {
        inline {
            val drive = mecanumSubsystem.value.mecanumDrive
            val startPoseLeft = Pose2d(-37.5, -62.0, Math.toRadians(90.0))

            drive.poseEstimate = startPoseLeft

            align_preload = drive.trajectoryBuilder(startPoseLeft)
                .lineTo(Vector2d(startPoseLeft.x - 1, -20.0))
                .build()

            preload_to_turn = drive.trajectorySequenceBuilder(align_preload.end())
                .lineTo(Vector2d(startPoseLeft.x - 1, -8.0))
                .build()

            turn_to_stack = drive.trajectorySequenceBuilder(preload_to_turn.end())
                .setAccelConstraint(accelConstraint)
                .setVelConstraint(slowConstraint)
                .turn(Math.toRadians(90.0))
                .resetConstraints()
                .build()

            go_to_stack = drive.trajectorySequenceBuilder(turn_to_stack.end())
                .setAccelConstraint(accelConstraint)
                .setVelConstraint(slowConstraint)
                .lineTo(Vector2d(-57.3, -9.5))
                .resetConstraints()
                .build()


            stack_to_high_1 = drive.trajectorySequenceBuilder(go_to_stack.end())
                .setReversed(true)
                .lineTo(Vector2d(-3.4, -9.0))
                .build()

            stack_to_high_2 = drive.trajectorySequenceBuilder(go_to_stack.end())
                .setReversed(true)
                .lineTo(Vector2d(-2.6, -9.0))
                .build()

            high_align_1 = drive.trajectorySequenceBuilder(stack_to_high_1.end())
                .setAccelConstraint(accelConstraint)
                .setVelConstraint(slowConstraint)
                .lineTo(Vector2d(-1.0, -9.0))
                .resetConstraints()
                .build()

            high_align_2 = drive.trajectorySequenceBuilder(stack_to_high_2.end())
                .setAccelConstraint(accelConstraint)
                .setVelConstraint(slowConstraint)
                .lineTo(Vector2d(-1.0, -9.0))
                .resetConstraints()
                .build()

            high_to_stack_1 = drive.trajectorySequenceBuilder(high_align_1.end())
                .lineTo(Vector2d(-55.75, -8.8))
                .setAccelConstraint(accelConstraint)
                .setVelConstraint(slowConstraint)
                .lineTo(Vector2d(-57.0, -8.8))
                .resetConstraints()
                .build()

            high_to_stack_2 = drive.trajectorySequenceBuilder(high_align_2.end())
                .lineTo(Vector2d(-55.75, -9.5))
                .setAccelConstraint(accelConstraint)
                .setVelConstraint(slowConstraint)
                .lineTo(Vector2d(-58.75, -9.5))
                .resetConstraints()
                .build()

        }
    }

    override fun getControlScheme(): ContinuousReuseCommand = newContinuous {
    }

    override fun getStartRoutine(): SerialCommand = newSerial {
        run(SetClawCommand(SetClawCommand.ClawState.CLOSED))
        sleep(200)
        run(GoToLevelCommand(400, .3))
        parallel(ParallelCommand.Behavior.DEFAULT) {
            run(TrajectoryCommand({align_preload}))
            run(GoToAngleSensorCommand(90.0, .3))
        }
        sleep(250)
        run(GoToLevelCommand(300, .3))
        run(SetClawCommand(SetClawCommand.ClawState.OPEN))
        parallel(ParallelCommand.Behavior.DEFAULT) {
            run(TrajectoryCommand(trajectorySequence = {preload_to_turn}))
            serial {
                sleep(100)
                run(GoToLevelCommand(70, .3))
            }
        }
        run(GoToAngleSensorCommand(0.0, .1))
        sleep(300)
        run(TrajectoryCommand(trajectorySequence = {go_to_stack}))
        sleep(200)
        run(SetClawCommand(SetClawCommand.ClawState.CLOSED))
        sleep(300)
        run(GoToLevelCommand(200, .3))
        sleep(200)
        parallel(ParallelCommand.Behavior.DEFAULT) {
            run(TrajectoryCommand(trajectorySequence = {stack_to_high_1}))
            serial {
                sleep(200)
                run(GoToLevelCommand(370, .3))
                run(GoToAngleSensorCommand(HIGH_TURRET_ANGLE, .15, 160))
            }
        }
        run(TrajectoryCommand(trajectorySequence = {high_align_1}))
        parallel(ParallelCommand.Behavior.DEFAULT) {
            run(GoToLevelCommand(700, .3))
            serial {
                sleep(1500)
                run(GoToLevelCommand(375, .3))
                sleep(500)
                run(SetClawCommand(SetClawCommand.ClawState.OPEN))
                sleep(100)
            }
        }
        sleep(100)
        parallel(ParallelCommand.Behavior.DEFAULT) {
            run(TrajectoryCommand(trajectorySequence = {high_to_stack_1}))
            serial {
                run(GoToAngleCommand(0.0, .3))
                sleep(500)
                run(GoToLevelCommand(50, .3))
            }
        }
        run(SetClawCommand(SetClawCommand.ClawState.CLOSED))
        sleep(200)
        run(GoToLevelCommand(200, .3))
        sleep(200)
        parallel(ParallelCommand.Behavior.DEFAULT) {
            run(TrajectoryCommand(trajectorySequence = {stack_to_high_2}))
            serial {
                sleep(200)
                run(GoToLevelCommand(300, .3))
                run(GoToAngleCommand(HIGH_TURRET_ANGLE, .1))
            }
        }
        run(TrajectoryCommand(trajectorySequence = {high_align_2}))
        parallel(ParallelCommand.Behavior.DEFAULT) {
            run(GoToLevelCommand(700, .3))
            serial {
                sleep(1500)
                run(GoToLevelCommand(350))
                sleep(300)
                run(SetClawCommand(SetClawCommand.ClawState.OPEN))
            }
        }

    }

    companion object {
        const val HIGH_TURRET_ANGLE = -100.0
    }

}