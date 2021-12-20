package org.firstinspires.ftc.teamcode

import com.acmerobotics.roadrunner.control.PIDCoefficients
import com.acmerobotics.roadrunner.control.PIDFController
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive
import org.firstinspires.ftc.teamcode.stateMachine.StateMachineBuilder
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence
@Autonomous(preselectTeleOp = "CompTeleOp")
class DepositTopBlueAuto : OpMode() {
    private val startX = 15.0
    private val startY = 56.0
    private val startAngle = Math.toRadians(90.0)
    private val depositServoAngle = 0.6
    private val depositArmAngle = 140.0
    private val depositX = -11.0
    private val depositY = 35.0
    private val depositBotAngle = 90.0
    private val arm = Arm()


    private val startPose = Pose2d(startX,startY,startAngle)

    private val depositPose = Pose2d(depositX,depositY,depositBotAngle)

    private lateinit var moveToDepositTrajectorySequence : TrajectorySequence

    private lateinit var moveToWarehouseFrontTrajectorySequence : TrajectorySequence

    private lateinit var moveIntoWarehouseTrajectorySequence : TrajectorySequence

    private fun moveArmToDegree(degrees: Double) {

    }

    lateinit var outtakeServo: Servo


    fun moveOuttakeToOut(){
        outtakeServo.position = 0.6

    }

    fun moveOuttakeToOpen(){
        outtakeServo.position = 0.92

    }



    lateinit var drive: SampleMecanumDrive


    private enum class InitialDepositStates {
        MOVE_TO_DEPOSIT,
        MOVE_ARM_UP,
        MOVE_OUTTAKE,
        MOVE_ARM_DOWN,
        GO_TO_FRONT_WAREHOUSE,
        GO_INTO_WAREHOUSE
    }

    private val initialDepositStateMachine = StateMachineBuilder<InitialDepositStates>()
            .state(InitialDepositStates.MOVE_TO_DEPOSIT)
            .onEnter { drive.followTrajectorySequenceAsync(moveToDepositTrajectorySequence) }
            .transition { !drive.isBusy }
            .state(InitialDepositStates.MOVE_ARM_UP)
            .onEnter { arm.moveArmToTopPos() }
            .transitionTimed(2.0)
            .state (InitialDepositStates.MOVE_OUTTAKE)
            .onEnter { moveOuttakeToOut() }
            .transitionTimed( 2.0)
            .state(InitialDepositStates.MOVE_ARM_DOWN)
            .onEnter{
                arm.moveArmToBottomPos()
                moveOuttakeToOpen()
            }
            .transitionTimed(2.0)
            .state (InitialDepositStates.GO_TO_FRONT_WAREHOUSE)
            .onEnter {drive.followTrajectorySequenceAsync(moveToWarehouseFrontTrajectorySequence) }
            .transition { !drive.isBusy }

            .state (InitialDepositStates.GO_INTO_WAREHOUSE)
            .onEnter {drive.followTrajectorySequenceAsync(moveIntoWarehouseTrajectorySequence)}
            .transition { !drive.isBusy }
            .build()


    override fun init() {
        drive = SampleMecanumDrive(hardwareMap)
        arm.init(hardwareMap)
        outtakeServo = hardwareMap.get(Servo::class.java, "Outtake") as Servo
        outtakeServo.position = 0.83
        moveToDepositTrajectorySequence = drive.trajectorySequenceBuilder(startPose)
                .lineToConstantHeading(Vector2d(-9.0,37.0) )
                .build()
        moveToWarehouseFrontTrajectorySequence = drive.trajectorySequenceBuilder(depositPose)
                .splineTo(Vector2d(15.0,60.0),0.0)
                .build()
        moveIntoWarehouseTrajectorySequence = drive.trajectorySequenceBuilder(moveToWarehouseFrontTrajectorySequence.end())
                .lineToConstantHeading(Vector2d(41.0,63.0))
                .build()

        drive.poseEstimate = startPose


        initialDepositStateMachine.start()
    }



    override fun loop() {
        initialDepositStateMachine.update()
        drive.update()
        arm.update()
    }
}