package org.firstinspires.ftc.teamcode

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.control.PIDCoefficients
import com.acmerobotics.roadrunner.control.PIDFController
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.HardwareMap

class Arm {

    companion object {
        @JvmStatic var kp = 0.015
        @JvmStatic var ki = 0.0
        @JvmStatic var kd = 0.00060
        @JvmStatic var targetAngle = 0.0
        @JvmStatic var kcos = 0.275
        @JvmStatic var kv = 0.0
        @JvmStatic var depositAngle = 140.0
        @JvmStatic var restAngle = -55.0
        @JvmStatic var sharedAngle = 195.0
    }
    lateinit var arm: DcMotor

    var degreesPerTick = 90 / 184.0
    var ticksPerDegree = 184.0 / 90
    var targetTicks = 0.0
    var output = 0.0
    var pidOutput = 0.0
    var feedForward = 0.0

    var currentPosition = (arm.currentPosition - 114).toDouble()

    private fun moveArmToDegree(degrees: Double) {
        targetAngle = degrees
        targetTicks = targetAngle * ticksPerDegree
        armController.reset()
        armController.targetPosition = targetTicks
    }

    fun moveArmToMidPos(){

        moveArmToDegree(-20.0)
    }

    fun moveArmToTopPos(){

        moveArmToDegree(140.0)
    }

    fun moveArmToTopPosTwo(){

        moveArmToDegree(135.0)
    }

    fun moveArmToBottomPos(){

        moveArmToDegree(-55.0)
    }


    fun update()
    {
        currentPosition = (arm.currentPosition - 114).toDouble()

        feedForward = getFeedForward(Math.toRadians(targetAngle))
        pidOutput = armController.update(currentPosition)
        output = feedForward + pidOutput

        arm.power = output
    }

    fun updateTelemetry() {
        val packet = TelemetryPacket()
        packet.put("target angle", LeviTeleOp.targetAngle)
        packet.put("output", output)
        packet.put("Current Position", currentPosition)
        packet.put("degrees", currentPosition * degreesPerTick)
        packet.put("feedforward", getFeedForward(Math.toRadians(LeviTeleOp.targetAngle)))
        packet.put("target ticks", targetTicks)
        FtcDashboard.getInstance().sendTelemetryPacket(packet)
    }



    var armController = PIDFController(PIDCoefficients(kp, ki, kd))

    private fun getFeedForward(targetAngle: Double): Double {
        return Math.cos(targetAngle) * kcos
    }

    fun init (hardwareMap: HardwareMap) {
        arm = hardwareMap.dcMotor["Arm"]

        arm.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        arm.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        arm.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.FLOAT

        armController.reset()
        LeviTeleOp.targetAngle = LeviTeleOp.restAngle
        targetTicks = LeviTeleOp.restAngle * ticksPerDegree
        armController.targetPosition = targetTicks

    }


}
