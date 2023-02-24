package org.firstinspires.ftc.teamcode.commands

import com.acmerobotics.roadrunner.trajectory.Trajectory
import org.firstinspires.ftc.teamcode.moonshine.Command
import org.firstinspires.ftc.teamcode.moonshine.SharedSubsystem
import org.firstinspires.ftc.teamcode.subsystems.MecanumSubsystem

class TrajectoryCommand(private val trajectory: Trajectory) : Command() {
    val mecanumSubsystem = SharedSubsystem(MecanumSubsystem::class.java)

    override fun onStart() {
        mecanumSubsystem.value.mecanumDrive.followTrajectoryAsync(trajectory)
    }

    override fun onTick() {
        if(!mecanumSubsystem.value.mecanumDrive.isBusy)
            end()
    }

    override fun onEnd() {

    }
}