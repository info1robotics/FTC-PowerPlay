package org.firstinspires.ftc.teamcode.subsystems

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.teamcode.moonshine.Subsystem

class TurretSubsystem : Subsystem() {

    private lateinit var turretMotor: DcMotorEx
    private lateinit var brakeServo: Servo
    private lateinit var superBrakeServo: Servo
    var noAutoBrakes = false

    val hasReachedTarget: Boolean get() = !turretMotor.isBusy

    var targetPower: Double = 0.0

    val currentPosition: Int
        get() = turretMotor.currentPosition

    var targetPosition: Int
        get() = turretMotor.targetPosition
        set(value) { turretMotor.targetPosition = value }

    val currentAngle: Double
    get() = (turretMotor.getCurrentPosition() * 360 * GEAR_RATIO) / TICKS_PER_REVOLUTION

    fun engageBrake() {
        brakeServo.position = BRAKE_OPEN
    }

    fun disengageBrake() {
        brakeServo.position = BRAKE_CLOSED
    }

    fun engageSuperBrake() {
        superBrakeServo.position = SUPER_BRAKE_OPEN
    }

    fun disengageSuperBrake() {
        superBrakeServo.position = SUPER_BRAKE_CLOSED
    }

    fun setTargetAngle(desiredAngle: Double) {
        var finalAngle = desiredAngle
        if (desiredAngle > MAX_ANGLE) finalAngle = MAX_ANGLE
        targetPosition = ((TICKS_PER_REVOLUTION / GEAR_RATIO) / (360 / finalAngle)).toInt()
        turretMotor.mode = DcMotor.RunMode.RUN_TO_POSITION
        disengageBrake()
        disengageSuperBrake()
    }


    override fun onStart() {
        turretMotor = hardwareMap.get(DcMotorEx::class.java, "Turret")
        turretMotor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        turretMotor.targetPosition = 0
        turretMotor.mode = DcMotor.RunMode.RUN_TO_POSITION
        turretMotor.direction = DcMotorSimple.Direction.FORWARD
        brakeServo = hardwareMap.get(Servo::class.java, "BrakeServo2")
        superBrakeServo = hardwareMap.get(Servo::class.java, "BrakeServo1")
    }

    override fun onTick() {
        telemetry.addData("Current Position ", currentPosition)
        telemetry.addData("Target Position ", targetPosition)
        telemetry.addData("POWR ", targetPower)
        if(!noAutoBrakes) {
            if (hasReachedTarget) {
                targetPower = 0.0
                engageBrake()
                engageSuperBrake()
            } else {
                disengageBrake()
                disengageSuperBrake()
            }
        }
        turretMotor.power = targetPower
    }

    override fun onEnd() {

    }

    companion object {
        const val BRAKE_OPEN = 0.35
        const val BRAKE_CLOSED = 0.6
        const val SUPER_BRAKE_OPEN = 0.3
        const val SUPER_BRAKE_CLOSED = 0.55
        const val GEAR_RATIO = 2.0
        const val MIN_TICK = -16000
        const val MAX_TICK = 16000
        const val MIN_ANGLE = -220.0
        const val MAX_ANGLE = 220.0
        const val TICKS_PER_REVOLUTION = 3895.9
        const val ANGLE_THRESHOLD = 1.0
    }
}