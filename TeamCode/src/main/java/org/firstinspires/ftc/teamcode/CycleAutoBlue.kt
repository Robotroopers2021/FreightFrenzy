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
import org.firstinspires.ftc.teamcode.util.math.Pose

@Autonomous(preselectTeleOp = "CompTeleOp")
class CycleAutoBlue : OpMode() {
    private val startX = 15.0
    private val startY = 56.0
    private val startAngle = Math.toRadians(90.0)
    private val depositServoAngle = 0.6
    private val depositArmAngle = 140.0
    private val depositX = -11.0
    private val depositY = 35.0
    private val depositBotAngle = 90.0
    private val cycleBegPoseX = 45.0
    private val cycleBegPoseY = 63.0
    private val cycleBegPoseAngle = Math.toRadians(0.0)
    private val warehouseFrontX = 6.0
    private val warehouseFrontY = 63.0
    private val warehouseFrontAngle = Math.toRadians(0.0)
    private val arm = Arm()


    private val startPose = Pose2d(startX,startY,startAngle)

    private val depositPose = Pose2d(depositX,depositY,depositBotAngle)

    private val cycleBegPose = Pose2d(cycleBegPoseX,cycleBegPoseY,cycleBegPoseAngle)

    private val warehouseFrontPose = Pose2d(warehouseFrontX,warehouseFrontY,warehouseFrontAngle)

    private lateinit var moveToDepositTrajectorySequence : TrajectorySequence

    private lateinit var moveIntoWarehouseFrontTrajectorySequence : TrajectorySequence

    private lateinit var moveBackOutTrajectorySequence : TrajectorySequence

    private lateinit var moveToDepositTwoTrajectorySequence : TrajectorySequence

    private lateinit var moveIntoWarehouseEndTrajectorySequence : TrajectorySequence

    private fun moveArmToDegree(degrees: Double) {

    }

    lateinit var outtakeServo: Servo

    lateinit var intakeMotor: DcMotor


    fun moveOuttakeToOut(){
        outtakeServo.position = 0.6

    }

    fun moveOuttakeToLock(){
        outtakeServo.position = 0.83
    }

    fun moveOuttakeToOpen(){
        outtakeServo.position = 0.92

    }

    fun intakeFreight(){
        intakeMotor.power = -1.0
    }

    fun stopIntake(){
        intakeMotor.power = 0.0
    }

    fun getFreightOut(){
        intakeMotor.power = 1.0
    }



    lateinit var drive: SampleMecanumDrive


    private enum class InitialDepositStates {
        MOVE_TO_DEPOSIT_ARM__UP,
        MOVE_OUTTAKE,
        GO_INTO_WAREHOUSE,
        INTAKE_FREIGHT,
        GO_BACK_OUT,
        MOVE_TO_DEPOSIT_TWO,
        MOVE_ARM_UP_TWO,
        MOVE_OUTTAKE_TWO,
        GO_INTO_WAREHOUSE_END,
    }

    private val initialDepositStateMachine = StateMachineBuilder<InitialDepositStates>()

            .state(InitialDepositStates.MOVE_TO_DEPOSIT_ARM__UP)
            .onEnter {
                drive.followTrajectorySequenceAsync(moveToDepositTrajectorySequence)
                arm.moveArmToTopPos()
            }
            .transition { !drive.isBusy }

            .state (InitialDepositStates.MOVE_OUTTAKE)
            .onEnter { moveOuttakeToOut() }
            .transitionTimed( 0.2)

            .state (InitialDepositStates.GO_INTO_WAREHOUSE)
            .onEnter {
                drive.followTrajectorySequenceAsync(moveIntoWarehouseFrontTrajectorySequence)
                arm.moveArmToBottomPos()
                moveOuttakeToOpen()
                intakeFreight()
            }
            .transition { !drive.isBusy }

            .state (InitialDepositStates.INTAKE_FREIGHT)
            .onEnter {
                intakeFreight()
            }
            .transitionTimed(0.5)

            .state (InitialDepositStates.GO_BACK_OUT)
            .onEnter {
                drive.followTrajectorySequenceAsync(moveBackOutTrajectorySequence)
                getFreightOut()
            }
            .transition { !drive.isBusy}

            .state (InitialDepositStates.MOVE_TO_DEPOSIT_TWO)
            .onEnter {
                drive.followTrajectorySequenceAsync(moveToDepositTwoTrajectorySequence)
                moveOuttakeToLock()
                stopIntake()
            }
            .transition {!drive.isBusy}

            .state (InitialDepositStates.MOVE_ARM_UP_TWO)
            .onEnter {
                arm.moveArmToTopPosTwo()
            }
            .transitionTimed(0.8)

            .state (InitialDepositStates.MOVE_OUTTAKE_TWO)
            .onEnter{
                moveOuttakeToOut()
            }
            .transitionTimed(0.2)


            .state (InitialDepositStates.GO_INTO_WAREHOUSE_END)
            .onEnter {
                drive.followTrajectorySequenceAsync(moveIntoWarehouseEndTrajectorySequence)
                arm.moveArmToBottomPos()
                moveOuttakeToOpen()
            }
            .transition{!drive.isBusy}

            .build()


    override fun init() {
        drive = SampleMecanumDrive(hardwareMap)
        arm.init(hardwareMap)
        outtakeServo = hardwareMap.get(Servo::class.java, "Outtake") as Servo
        outtakeServo.position = 0.83
        intakeMotor = hardwareMap.dcMotor["Intake"]
        intakeMotor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        moveToDepositTrajectorySequence = drive.trajectorySequenceBuilder(startPose)
                .lineToConstantHeading(Vector2d(-9.0,37.0) )
                .build()
        moveIntoWarehouseFrontTrajectorySequence = drive.trajectorySequenceBuilder(depositPose)
                .splineTo(Vector2d(15.0,63.0),0.0)
                .lineToConstantHeading(Vector2d(51.0,63.0))
                .build()
        moveBackOutTrajectorySequence = drive.trajectorySequenceBuilder(cycleBegPose)
                .lineToConstantHeading(Vector2d (6.0,63.0))
                .build()
        moveToDepositTwoTrajectorySequence = drive.trajectorySequenceBuilder(warehouseFrontPose)
                .splineToSplineHeading(Pose2d(-9.0, 41.0, Math.toRadians(90.0)), Math.toRadians(60.0))
                .build()
        moveIntoWarehouseEndTrajectorySequence = drive.trajectorySequenceBuilder(depositPose)
                .splineTo(Vector2d(15.0,65.0),0.0)
                .lineToConstantHeading(Vector2d(41.0,65.0))
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