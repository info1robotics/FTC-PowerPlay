package org.firstinspires.ftc.teamcode.commands

import com.qualcomm.robotcore.hardware.DistanceSensor
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.teamcode.moonshine.Command
import org.firstinspires.ftc.teamcode.moonshine.SharedSubsystem
import org.firstinspires.ftc.teamcode.moonshine.builtin.BlueprintCommand
import org.firstinspires.ftc.teamcode.moonshine.builtin.ParallelCommand
import org.firstinspires.ftc.teamcode.moonshine.extensions.*
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem
import kotlin.math.sign

class GoToAngleSensorCommand(private val angle: Double, private val speed: Double) : BlueprintCommand() {

    val distanceSensor by InjectHardware<DistanceSensor>("DistanceSensor")
    val turretSubsystem = SharedSubsystem(TurretSubsystem::class.java)

    override fun getBlueprint(): Command = newSerial {
        parallel(ParallelCommand.Behavior.SHORT) {
            run(GoToAngleCommand(angle, speed))
            run(object : Command() {
                override fun onStart() {}
                override fun onTick() {
                    telemetry.addData("Distance", distanceSensor.getDistance(DistanceUnit.MM))
                    if(distanceSensor.getDistance(DistanceUnit.MM) <= 300) {
                        end()
                    }
                }
                override fun onEnd() {}
            })
        }
        sleep(100)
        run(GoToTickCommand(
            {turretSubsystem.value.currentPosition - 10 * sign(angle).toInt()},
            0.3))
    }

}