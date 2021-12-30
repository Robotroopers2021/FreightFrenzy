package org.firstinspires.ftc.teamcode


import com.qualcomm.hardware.rev.Rev2mDistanceSensor
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.teamcode.stateMachine.StateMachineBuilder

class IntakeSequence(
    private val intakeMotor: DcMotor,
    val outtakeServo: Servo,
    private val distanceSensor: Rev2mDistanceSensor,
    val arm : Arm,
){

    private fun intakeFreight(){
        intakeMotor.power = -1.0
    }

    private fun stopIntake(){
        intakeMotor.power = 0.0
    }

    private fun moveOuttakeToLock(){
        outtakeServo.position = 0.83
    }

    fun runIntakeSequence(shouldStart : Boolean) {
        if (shouldStart && intakeSequence.running) {
            intakeSequence.stop()
        }

        else if (shouldStart && !intakeSequence.running){
            intakeSequence.reset()
            intakeSequence.start()
        }
        else if (!shouldStart && !intakeSequence.running){
            intakeSequence.update()
        }
        else if (!shouldStart && intakeSequence.running) {
            intakeSequence.update()
        }
    }


    private enum class IntakeSequenceStates {
        INTAKE,
        LOCK_INDEXER,
        ARM_MID,
    }
    private val intakeSequence = StateMachineBuilder<IntakeSequenceStates>()
        .state(IntakeSequenceStates.INTAKE)
        .onEnter {
            intakeFreight()
        }
        .transition {
            val value = distanceSensor.getDistance(DistanceUnit.INCH)
            value < 6.0
        }
        .state(IntakeSequenceStates.LOCK_INDEXER)
        .onEnter {
            stopIntake()
            moveOuttakeToLock()
        }
        .transitionTimed (0.5)
        .state(IntakeSequenceStates.ARM_MID)
        .onEnter {
            arm.moveArmToMidPos()
        }

        .build()

}