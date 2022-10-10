package org.firstinspires.ftc.teamcode.EOCV

import android.os.SystemClock
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.openftc.easyopencv.OpenCvCamera
import org.openftc.easyopencv.OpenCvCameraFactory
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.openftc.easyopencv.OpenCvCamera.AsyncCameraOpenListener
import org.openftc.easyopencv.OpenCvCameraRotation

open class AprilTagDetection(var opMode: LinearOpMode) {
    private val fx = 578.272
    private val fy = 578.272
    private val cx = 402.145
    private val cy = 221.506
    private val tagSize = 0.166
    private var zone = 0
    private var tagInView = 0
    private var parkZone = ""
    val camera: OpenCvCamera
    private val aprilTagDetectionPipeline: AprilTagDetectionPipeline

    init {
        val cameraMonitorViewId = opMode.hardwareMap.appContext.resources.getIdentifier("cameraMonitorViewId", "id", opMode.hardwareMap.appContext.packageName)
        camera = OpenCvCameraFactory.getInstance().createWebcam(opMode.hardwareMap.get(WebcamName::class.java, "Webcam 1"), cameraMonitorViewId)
        aprilTagDetectionPipeline = AprilTagDetectionPipeline(tagSize, fx, fy, cx, cy)
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
                    17 -> zone = 1
                    18 -> zone = 2
                    19 -> zone = 3
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