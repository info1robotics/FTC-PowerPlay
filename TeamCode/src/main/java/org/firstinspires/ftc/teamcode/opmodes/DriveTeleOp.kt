package org.firstinspires.ftc.teamcode.opmodes

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.moonshine.extensions.add
import org.firstinspires.ftc.teamcode.moonshine.extensions.subsystemOfType
import org.firstinspires.ftc.teamcode.moonshine.extensions.subsystems
import org.firstinspires.ftc.teamcode.subsystems.DrivetrainSubsystem
import org.firstinspires.ftc.teamcode.subsystems.LocalizationSubsystem

@TeleOp
class DriveTeleOp : LinearOpMode() {

    private val subsystems = subsystems {
        add(subsystemOfType<LocalizationSubsystem>())
        add(subsystemOfType<DrivetrainSubsystem>())
    }

    override fun runOpMode() {
        while(!opModeInInit()) {
            subsystems.initStep()
        }
        subsystems.end()
        while(opModeIsActive() && !isStopRequested) {
            subsystems.step()
        }
    }


}