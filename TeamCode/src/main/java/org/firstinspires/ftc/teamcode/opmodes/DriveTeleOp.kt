package org.firstinspires.ftc.teamcode.opmodes

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.moonshine.Command
import org.firstinspires.ftc.teamcode.moonshine.extensions.*
import org.firstinspires.ftc.teamcode.subsystems.DrivetrainSubsystem
import org.firstinspires.ftc.teamcode.subsystems.LocalizationSubsystem

@TeleOp
class DriveTeleOp : LinearOpMode() {
    private val localizationSubsystem by InjectSubsystem(LocalizationSubsystem(Pose2d(.0,.0,.0)))
    private val drivetrainSubsystem by InjectSubsystem(DrivetrainSubsystem())

    private val subsystems = subsystems {
        add(localizationSubsystem)
        add(drivetrainSubsystem)
    }

    private val controlScheme = newTeleOpRoutine {
        trigger({ gamepad1.y }) {
            serial {
                inline { telemetry.addLine("yup it works") }
                sleep(1000)
                inline { telemetry.addLine("100%") }
            }
        }
    }

    override fun runOpMode() {

        while(!opModeInInit()) {
            subsystems.step()
        }
        subsystems.end()

        while(opModeIsActive() && !isStopRequested) {
            subsystems.step()
            controlScheme.step()
        }
        subsystems.end()
        controlScheme.end()
    }


}