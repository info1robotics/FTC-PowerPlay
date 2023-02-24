package org.firstinspires.ftc.teamcode.subsystems

import org.firstinspires.ftc.teamcode.moonshine.Subsystem
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive

class MecanumSubsystem : Subsystem() {
    lateinit var mecanumDrive: SampleMecanumDrive

    override fun onStart() {
        mecanumDrive = SampleMecanumDrive(hardwareMap)
    }

    override fun onTick() {
        mecanumDrive.update()
    }

    override fun onEnd() {
    }
}