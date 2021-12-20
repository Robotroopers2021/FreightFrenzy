package org.firstinspires.ftc.teamcode

import com.acmerobotics.roadrunner.control.PIDCoefficients
import com.acmerobotics.roadrunner.control.PIDFController
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.HardwareMap

class Arm {

    companion object {
        @JvmStatic var kp = 0.015
        @JvmStatic var ki = 0.0
        @JvmStatic var kd = 0.00075
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

    private fun moveArmToDegree(degrees: Double) {
        targetAngle = degrees
        targetTicks = targetAngle * ticksPerDegree
        armController.reset()
        armController.targetPosition = targetTicks
    }

    fun moveArmToTopPos(){

        moveArmToDegree(140.0)
    }

    fun moveArmToBottomPos(){

        moveArmToDegree(-55.0)
    }


    fun update ()
    {
        val currentPosition = (arm.currentPosition - 114).toDouble()

        feedForward = getFeedForward(Math.toRadians(targetAngle))
        pidOutput = armController.update(currentPosition)
        output = feedForward + pidOutput

        arm.power = output
    }





    var armController = PIDFController(PIDCoefficients(kp, ki, kd))

    private fun getFeedForward(targetAngle: Double): Double {
        return Math.cos(targetAngle) * kcos
    }

    fun init (hardwareMap: HardwareMap) {
        arm = hardwareMap.dcMotor["Arm"]
    }


}
