package org.firstinspires.ftc.teamcode


import com.qualcomm.hardware.rev.Rev2mDistanceSensor
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.teamcode.stateMachine.StateMachineBuilder

class IntakeSequence : OpMode() {

    private val arm = Arm()

    private lateinit var distanceSensor : Rev2mDistanceSensor

    private lateinit var intakeMotor: DcMotor

    private lateinit var outtakeServo: Servo

    private fun intakeFreight(){
        intakeMotor.power = -1.0
    }

    private fun stopIntake(){
        intakeMotor.power = 0.0
    }

    private fun moveOuttakeToLock(){
        outtakeServo.position = 0.83
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



    override fun init() {
        intakeMotor = hardwareMap.dcMotor["Intake"]
        intakeMotor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        distanceSensor = hardwareMap.get(Rev2mDistanceSensor::class.java, "distanceSensor") as Rev2mDistanceSensor
        outtakeServo = hardwareMap.get(Servo::class.java, "Outtake") as Servo
        intakeSequence.start()
    }

    override fun loop() {
        intakeSequence.update()
        arm.update()
    }
}