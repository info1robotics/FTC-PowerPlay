package org.firstinspires.ftc.teamcode.subsystems

import org.firstinspires.ftc.teamcode.moonshine.Subsystem
import org.firstinspires.ftc.teamcode.moonshine.annotations.RequireHardware
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotor.RunMode
import com.qualcomm.robotcore.hardware.DcMotorSimple
import org.firstinspires.ftc.teamcode.moonshine.CommandEnv
import org.firstinspires.ftc.teamcode.subsystems.LinkageSubsystem
import org.firstinspires.ftc.teamcode.moonshine.OpModeType
import org.firstinspires.ftc.teamcode.moonshine.extensions.InjectHardware

class LinkageSubsystem : Subsystem() {
    private val linkageLeft:
        DcMotorEx by InjectHardware("LinkageLeft")

    private val linkageRight:
        DcMotorEx by InjectHardware("LinkageRight")


    val hasReachedTarget: Boolean get() = !linkageLeft.isBusy || !linkageRight.isBusy

    override fun onStart() {
        linkageLeft.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        linkageRight.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE

        linkageLeft.direction = DcMotorSimple.Direction.FORWARD
        linkageRight.direction = DcMotorSimple.Direction.REVERSE

        setMode(RunMode.RUN_USING_ENCODER)
    }

    override fun onTick() {

    }

    fun setMode(runMode: RunMode) {
        linkageLeft.mode = runMode
        linkageRight.mode = runMode
    }

    fun setPowers(pw: Double) {
        linkageRight.power = pw
        linkageLeft.power = pw
    }

    fun setTargetPosition(pos: Int) {
        linkageLeft.targetPosition = pos
        linkageRight.targetPosition = pos
    }

    override fun onEnd() {}

    fun debug() {
        CommandEnv.getInstance().telemetry.addData("Left Linkage Tick Count ", linkageLeft.currentPosition)
        CommandEnv.getInstance().telemetry.addData("Right Linkage Tick Count ", linkageRight.currentPosition)
    }

    companion object {
        const val GROUND_LEVEL = -10
        const val JUNCTION_LEVEL = 100
        const val LOW_LEVEL = 250
        const val MID_LEVEL = 400
        const val HIGH_LEVEL = 550
        const val LINKAGE_THRESHOLD = 8
        const val SAFETY_THRESHOLD = 50
        var LINKAGE_MIN = -100
        var LINKAGE_MAX = 725
        var DESIRED_HEIGHT = 0
    }
}