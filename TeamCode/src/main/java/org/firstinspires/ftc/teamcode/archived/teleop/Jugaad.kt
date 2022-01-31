package org.firstinspires.ftc.teamcode.archived.teleop


import com.qualcomm.hardware.rev.Rev2mDistanceSensor
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.teamcode.stateMachine.StateMachineBuilder

class Jugaad(
    private val intakeMotor: DcMotor,
    val outtakeServo: Servo,
    private val distanceSensor: Rev2mDistanceSensor,
    val arm : Arm,
){

    fun intakeFreight(){
        intakeMotor.power = -0.75
    }

    fun stopIntake(){
        intakeMotor.power = 0.0
    }

    fun reverseIntake(){
        intakeMotor.power = 1.0
    }

    fun moveOuttakeToOpen(){
        outtakeServo.position = 0.90
    }

    fun moveOuttakeToLock(){
        outtakeServo.position = 0.80
    }

    fun moveOuttakeToDeposit(){
        outtakeServo.position = 0.6
    }

    fun runIntakeSequence(shouldStart : Boolean) {
        if (shouldStart && intakeSequence.running) {
            intakeSequence.stop()
        }

        else if (shouldStart && !intakeSequence.running){
            intakeSequence.reset()
            intakeSequence.start()
        }
        else {
            intakeSequence.update()
        }
    }


    private enum class IntakeSequenceStates {
        INTAKE,
        LOCK_INDEXER,
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

        .build()


    private enum class HighAutoOuttakeSequenceStates {
        MOVE_TO_DEPOSIT
    }

    private val HighAutoOuttakeSequence = StateMachineBuilder<HighAutoOuttakeSequenceStates>()
        .state(HighAutoOuttakeSequenceStates.MOVE_TO_DEPOSIT)
        .onEnter {
        }
}