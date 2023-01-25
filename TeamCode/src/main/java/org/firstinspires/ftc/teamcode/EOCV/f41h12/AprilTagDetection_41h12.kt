package org.firstinspires.ftc.teamcode.EOCV.f41h12

import android.os.SystemClock
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.openftc.easyopencv.OpenCvCamera
import org.openftc.easyopencv.OpenCvCameraFactory
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.openftc.easyopencv.OpenCvCamera.AsyncCameraOpenListener
import org.openftc.easyopencv.OpenCvCameraRotation

open class AprilTagDetection_41h12(var opMode: LinearOpMode) {
        
    private val fx = 578.272
    private val fy = 578.272
    private val cx = 402.145
    private val cy = 221.506
    private val tagSize = 0.166
    public var zone = 0
    private var tagInView = 0
    private var parkZone = ""
    public val camera: OpenCvCamera
    private val aprilTagDetectionPipeline: AprilTagDetectionPipeline_41h12

    init {
        val cameraMonitorViewId = opMode.hardwareMap.appContext.resources.getIdentifier("cameraMonitorViewId", "id", opMode.hardwareMap.appContext.packageName)
        camera = OpenCvCameraFactory.getInstance().createWebcam(opMode.hardwareMap.get(WebcamName::class.java, "Webcam 1"), cameraMonitorViewId)
        aprilTagDetectionPipeline = AprilTagDetectionPipeline_41h12(tagSize, fx, fy, cx, cy)
        camera.setPipeline(aprilTagDetectionPipeline)
        camera.openCameraDeviceAsync(object : AsyncCameraOpenListener {
            override fun onOpened() {
                camera.startStreaming(800, 448, OpenCvCameraRotation.UPRIGHT)
            }

            override fun onError(errorCode: Int) {}
        })
        opMode.telemetry.msTransmissionInterval = 50
    }

    fun detectZone() {

        val currentDetections = aprilTagDetectionPipeline.latestDetections
        if (currentDetections.isNotEmpty()) {
            currentDetections.forEach {tag ->
                tagInView = tag.id
                when (tag.id) {
                    169 -> zone = 1
                    1311 -> zone = 2
                    1506 -> zone = 3
                }
            }
        }

        SystemClock.sleep(20)

        when(zone){
            1 -> parkZone = "LEFT"
            2 -> parkZone = "MIDDLE"
            3 -> parkZone = "RIGHT"
        }

        opMode.telemetry.addData("Ideal Park Location ", parkZone)
        opMode.telemetry.addData("Location ID Detected in View ", zone)
        opMode.telemetry.addData("April Tag ID in View ", tagInView)

        opMode.telemetry.update()
    }
}