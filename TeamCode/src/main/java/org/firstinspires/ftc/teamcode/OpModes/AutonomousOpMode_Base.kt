package org.firstinspires.ftc.teamcode.OpModes

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.Common.StateOpMode
import org.firstinspires.ftc.teamcode.EOCV.AprilTagDetection

@Autonomous(name = "AutonomousOpMode w/ AprilTag Detection")
class AutonomousOpMode_Base : StateOpMode() {
    @Throws(InterruptedException::class)
    override fun runOpMode() {

        val aprilTag = AprilTagDetection(this)

        // Initialization Loop (Replaces WaitForStart();
         while (!isStarted && !isStopRequested) {
             aprilTag.detectZone()
         }
    }
}