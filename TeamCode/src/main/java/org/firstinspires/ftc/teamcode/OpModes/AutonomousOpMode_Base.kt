package org.firstinspires.ftc.teamcode.OpModes

import com.outoftheboxrobotics.photoncore.PhotonCore
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.Common.StateOpMode
import org.firstinspires.ftc.teamcode.EOCV.f41h12.AprilTagDetection_41h12

@Autonomous(name = "AutonomousOpMode w/ AprilTag Detection")
class AutonomousOpMode_Base : StateOpMode() {
    @Throws(InterruptedException::class)
    override fun runOpMode() {

        PhotonCore.enable()
        val aprilTag = AprilTagDetection_41h12(this)
        var zone = 0;

        // Initialization Loop (Replaces WaitForStart();)
        
         while (!isStarted && !isStopRequested) {
             aprilTag.detectZone()
             if(aprilTag.zone!=0) zone = aprilTag.zone;
         }    }
}