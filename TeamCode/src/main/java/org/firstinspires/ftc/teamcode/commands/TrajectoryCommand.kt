package org.firstinspires.ftc.teamcode.commands

import com.acmerobotics.roadrunner.trajectory.Trajectory
import org.firstinspires.ftc.teamcode.moonshine.Command
import org.firstinspires.ftc.teamcode.moonshine.SharedSubsystem
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence
import org.firstinspires.ftc.teamcode.subsystems.MecanumSubsystem
import java.util.function.Supplier

class TrajectoryCommand(private val trajectory: Supplier<Trajectory>? = null,
                        private val trajectorySequence: Supplier<TrajectorySequence>? = null) : Command() {
    val mecanumSubsystem = SharedSubsystem(MecanumSubsystem::class.java)

    override fun onStart() {
        if(trajectory != null)
            mecanumSubsystem.value.mecanumDrive.followTrajectoryAsync(trajectory.get())
        else if(trajectorySequence != null)
            mecanumSubsystem.value.mecanumDrive.followTrajectorySequenceAsync(trajectorySequence.get())
    }

    override fun onTick() {
        if(!mecanumSubsystem.value.mecanumDrive.isBusy)
            end()
    }

    override fun onEnd() {

    }
}