package org.firstinspires.ftc.teamcode.archived.auto

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.teamcode.archived.teleop.Arm
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive
import org.firstinspires.ftc.teamcode.stateMachine.StateMachineBuilder
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence

@Autonomous(preselectTeleOp = "CompTeleOp")
class CycleAutoBlue : OpMode() {
    private val startX = 15.0
    private val startY = 63.0
    private val startAngle = Math.toRadians(0.0)
    private val depositX = -3.0
    private val depositY = 36.0
    private val depositBotAngle = 55.0
    private val warehouseFrontX = 44.0
    private val warehouseFrontY = 63.0
    private val warehouseFrontAngle = Math.toRadians(0.0)
    private val arm = Arm()


    private val startPose = Pose2d(startX,startY,startAngle)

    private val depositPose = Pose2d(depositX,depositY,depositBotAngle)

    private val warehouseFrontPose = Pose2d(warehouseFrontX,warehouseFrontY,warehouseFrontAngle)

    private val bruhPose = Pose2d(53.0, 65.0, Math.toRadians(0.0))

    private val idkPose = Pose2d(15.0, 63.0, Math.toRadians(0.0))

    private lateinit var moveToDepositTrajectorySequence : TrajectorySequence

    private lateinit var moveIntoWarehouseFrontTrajectorySequence : TrajectorySequence

    private lateinit var moveBackOutTrajectorySequence : TrajectorySequence

    private lateinit var moveToDepositTwoTrajectorySequence : TrajectorySequence

    private lateinit var moveIntoWarehouseEndTrajectorySequence : TrajectorySequence

    private lateinit var moveIntoWarehouseThree : TrajectorySequence

    private lateinit var moveToDepositThreeTrajectorySequence : TrajectorySequence

    private lateinit var moveBackOutThreeTrajectorySequence : TrajectorySequence

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
        GO_BACK_OUT_TWO,
        MOVE_TO_DEPOSIT_THREE,
        MOVE_ARM_UP_THREE,
        MOVE_OUTTAKE_THREE,
        GO_INTO_WAREHOUSE_REAL_END,
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
                drive.followTrajectorySequenceAsync(moveIntoWarehouseFrontTrajectorySequence)
                arm.moveArmToBottomPos()
                moveOuttakeToOpen()
                intakeFreight()
            }
            .transition{!drive.isBusy}

            .state (InitialDepositStates.GO_BACK_OUT_TWO)
            .onEnter {
                drive.followTrajectorySequenceAsync(moveBackOutTrajectorySequence)
                getFreightOut()
            }
            .transition { !drive.isBusy}

            .state (InitialDepositStates.MOVE_TO_DEPOSIT_THREE)
            .onEnter {
                drive.followTrajectorySequenceAsync(moveToDepositTwoTrajectorySequence)
                moveOuttakeToLock()
                stopIntake()
            }
            .transition {!drive.isBusy}

            .state (InitialDepositStates.MOVE_ARM_UP_THREE)
            .onEnter {
                arm.moveArmToTopPosTwo()
            }
            .transitionTimed(0.8)

            .state (InitialDepositStates.MOVE_OUTTAKE_THREE)
            .onEnter{
                moveOuttakeToOut()
            }
            .transitionTimed(0.2)

            .state(InitialDepositStates.GO_INTO_WAREHOUSE_REAL_END)
            .onEnter {
                drive.followTrajectorySequenceAsync(moveIntoWarehouseEndTrajectorySequence)
                arm.moveArmToBottomPos()
            }
            .transition {!drive.isBusy}
            .build()




    override fun init() {
        drive = SampleMecanumDrive(hardwareMap)
        arm.init(hardwareMap)
        outtakeServo = hardwareMap.get(Servo::class.java, "Outtake") as Servo
        outtakeServo.position = 0.83
        intakeMotor = hardwareMap.dcMotor["Intake"]
        intakeMotor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        moveToDepositTrajectorySequence = drive.trajectorySequenceBuilder(startPose)
                .setReversed(true)
                .splineToSplineHeading( Pose2d(-1.0, 37.0, Math.toRadians(55.0)), Math.toRadians(240.0))
                .setReversed(false)
                .build()
        moveIntoWarehouseFrontTrajectorySequence = drive.trajectorySequenceBuilder(Pose2d(-3.0,36.0, Math.toRadians(55.0)))
                .splineToSplineHeading( Pose2d( 15.0, 65.0, Math.toRadians(0.0)), Math.toRadians(0.0))
                .splineToConstantHeading( Vector2d(48.0, 65.0), Math.toRadians(0.0))
                .build()
        moveBackOutTrajectorySequence = drive.trajectorySequenceBuilder(warehouseFrontPose)
                .setReversed(true)
                .splineToConstantHeading( Vector2d(15.0, 63.0), Math.toRadians(180.0))
                .build()
        moveToDepositTwoTrajectorySequence = drive.trajectorySequenceBuilder(idkPose)
                .setReversed(true)
                .splineToSplineHeading( Pose2d(-1.0, 38.0, Math.toRadians(55.0)), Math.toRadians(240.0))
                .setReversed(false)
                .build()
        moveIntoWarehouseEndTrajectorySequence = drive.trajectorySequenceBuilder(Pose2d(-3.0,36.0, Math.toRadians(55.0)))
                .splineTo(Vector2d(15.0,65.0),0.0)
                .lineToConstantHeading(Vector2d(41.0,63.0))
                .build()
        moveIntoWarehouseThree = drive.trajectorySequenceBuilder(Pose2d(-3.0, 36.0, Math.toRadians(55.0)))
                .splineToSplineHeading( Pose2d( 15.0, 65.0, Math.toRadians(0.0)), Math.toRadians(0.0))
                .splineToConstantHeading( Vector2d(51.0, 65.0), Math.toRadians(0.0))
                .build()

        moveToDepositThreeTrajectorySequence = drive.trajectorySequenceBuilder(idkPose)
                .setReversed(true)
                .splineToSplineHeading( Pose2d(-1.0, 38.0, Math.toRadians(55.0)), Math.toRadians(240.0))
                .setReversed(false)
                .build()
        moveBackOutThreeTrajectorySequence = drive.trajectorySequenceBuilder(bruhPose)
                .setReversed(true)
                .splineToConstantHeading( Vector2d(15.0, 63.0), Math.toRadians(180.0))
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