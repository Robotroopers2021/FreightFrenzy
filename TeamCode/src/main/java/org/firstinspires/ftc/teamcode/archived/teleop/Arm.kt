package org.firstinspires.ftc.teamcode.archived.teleop

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.control.PIDCoefficients
import com.acmerobotics.roadrunner.control.PIDFController
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.HardwareMap
import kotlin.math.cos

class Arm {

    lateinit var arm: DcMotor

    fun init (hardwareMap: HardwareMap) {
        arm = hardwareMap.dcMotor["Arm"]

        arm.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        arm.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        arm.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.FLOAT

        armController.reset()
        targetAngle = restAngle
        targetTicks = restAngle * ticksPerDegree
        armController.targetPosition = targetTicks

    }

    companion object {
        @JvmStatic var kp = 0.015
        @JvmStatic var ki = 0.0
        @JvmStatic var kd = 0.00075
        @JvmStatic var targetAngle = 0.0
        @JvmStatic var kv = 0.0
        @JvmStatic var restAngle = -55.0
        @JvmStatic var kcos = 0.275
    }

    private var degreesPerTick = 90 / 184.0
    private var ticksPerDegree = 184.0 / 90
    private var targetTicks = 0.0
    var output = 0.0
    private var pidOutput = 0.0
    private var feedForward = 0.0

    private val currentPosition get() = (arm.currentPosition - 114).toDouble()

    private fun moveArmToDegree(degrees: Double) {
        targetAngle = degrees
        targetTicks = targetAngle * ticksPerDegree
        armController.reset()
        armController.targetPosition = targetTicks
    }

    fun moveArmToMidPos(){
        moveArmToDegree(134.0)
    }

    fun moveArmToTopPos(){
        moveArmToDegree(94.0)
    }

    fun moveArmToTopPosTwo(){
        moveArmToDegree(92.0)
    }

    fun moveArmToBottomPos(){
        moveArmToDegree(-55.0)
    }

    fun autoBottomPos(){
        moveArmToDegree(170.5)
    }

    fun moveArmToSharedPosBalanced() {
        moveArmToDegree(172.0)
    }

    fun moveArmToSharedPosTippedToAlliance() {
        moveArmToDegree(174.0)
    }

    fun moveArmToSharedPoseTippedToEnemy() {
        moveArmToDegree(169.0)
    }




    fun update()
    {

        feedForward = getFeedForward(Math.toRadians(targetAngle))
        pidOutput = armController.update(currentPosition)
        output = feedForward + pidOutput

        arm.power = output
    }

    fun updateTelemetry() {
        val packet = TelemetryPacket()
        packet.put("target angle", targetAngle)
        packet.put("output", output)
        packet.put("Current Position", currentPosition)
        packet.put("degrees", currentPosition * degreesPerTick)
        packet.put("feedforward", getFeedForward(Math.toRadians(targetAngle)))
        packet.put("target ticks", targetTicks)
        FtcDashboard.getInstance().sendTelemetryPacket(packet)
    }



    private var armController = PIDFController(PIDCoefficients(kp, ki, kd))

    private fun getFeedForward(targetAngle: Double): Double {
        return cos(targetAngle) * kcos
    }

}
