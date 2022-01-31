package org.firstinspires.ftc.teamcode.archived.auto


import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.qualcomm.hardware.rev.Rev2mDistanceSensor
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.teamcode.archived.teleop.Arm
import org.firstinspires.ftc.teamcode.archived.teleop.Jugaad
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive
import org.firstinspires.ftc.teamcode.stateMachine.StateMachineBuilder
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence

@Autonomous(preselectTeleOp = "CompTeleOp")
class GarbaageAuto : OpMode() {
    //Start Pose Values
    private val startX = 11.0
    private val startY = 57.25
    private val startAngle = Math.toRadians(90.0)
    //Deposit Pose Values
    private val depositX = 10.0
    private val depositY = 40.5
    private val depositAngle = Math.toRadians(90.0)
    // Warehouse One Pose Values
    private val warehouseOneX = 52.0
    private val warehouseOneY = 67.5
    private val warehouseOneAngle = Math.toRadians(0.0)
    //Warehouse Two Pose Values
    private val warehouseTwoX = 46.0
    private val warehouseTwoY = 65.5
    private val warehouseTwoAngle = Math.toRadians(0.0)
    //Arm.kt and Jugaad.kt instances
    private val arm = Arm()
    private lateinit var jugaad : Jugaad

    //Start Pose
    private val startPose = Pose2d(startX, startY, startAngle)
    //Deposit Pose
    private val depositPose = Pose2d(depositX, depositY, depositAngle)
    //Warehouse One Pose
    private val warehouseOnePose = Pose2d(warehouseOneX, warehouseOneY, warehouseOneAngle)
    //Warehouse Two Pose
    private val warehouseTwoPose = Pose2d(warehouseTwoX, warehouseTwoY, warehouseTwoAngle)
    //Timer
    private var motionTimer = ElapsedTime()
    //Servos, Motors, and Sensors
    private lateinit var outtakeServo: Servo
    private lateinit var intakeMotor: DcMotor
    private lateinit var distanceSensor : Rev2mDistanceSensor
    //Trajectory Sequences
    private lateinit var initialDepositTrajectoryTop : TrajectorySequence
    private lateinit var initialDepositTrajectoryMiddle : TrajectorySequence
    private lateinit var initialDepositTrajectoryBottom : TrajectorySequence
    private lateinit var moveIntoWarehouseOne : TrajectorySequence
    private lateinit var cycleDepositOne : TrajectorySequence
    private lateinit var moveIntoWarehouseTwo : TrajectorySequence
    private lateinit var cycleDepositTwo : TrajectorySequence
    private lateinit var parkAtEnd : TrajectorySequence
    //Sample Mecanum Drive
    lateinit var drive: SampleMecanumDrive

    //Initial Deposit States
    private enum class InitialDepositStates {
        INITIAL_DEPOSIT,
        MOVE_INTO_WAREHOUSE_ONE,
        CYCLE_DEPOSIT_ONE,
        MOVE_INTO_WAREHOUSE_TWO,
        CYCLE_DEPOSIT_TWO,
        PARK_AT_END,

    }
    //Initial Deposit State Machine
    private val initialDepositStateMachine = StateMachineBuilder<InitialDepositStates>()

        .state(InitialDepositStates.INITIAL_DEPOSIT)
        .onEnter{
            drive.followTrajectorySequenceAsync(initialDepositTrajectoryTop)
        }
//        .loop {
//            when(Globals.CUP_LOCATION) {
//                Webcam.CupStates.RIGHT -> drive.followTrajectorySequenceAsync(initialDepositTrajectoryTop)
//                Webcam.CupStates.MIDDLE -> drive.followTrajectorySequenceAsync(initialDepositTrajectoryMiddle)
//                Webcam.CupStates.LEFT -> drive.followTrajectorySequenceAsync(initialDepositTrajectoryBottom)
//            }
//        }
        .transition{!drive.isBusy}

        .state(InitialDepositStates.MOVE_INTO_WAREHOUSE_ONE)
        .onEnter{
            drive.followTrajectorySequenceAsync(moveIntoWarehouseOne)
            jugaad.arm.moveArmToBottomPos()
            jugaad.moveOuttakeToOpen()
            motionTimer.reset()
        }
        .transition{
            val value = distanceSensor.getDistance(DistanceUnit.INCH)
            value < 6.0 || motionTimer.seconds() > 5.0
        }

        .state(InitialDepositStates.CYCLE_DEPOSIT_ONE)
        .onEnter{
            drive.followTrajectorySequenceAsync(cycleDepositOne)
        }
        .transition{!drive.isBusy}

        .state(InitialDepositStates.MOVE_INTO_WAREHOUSE_TWO)
        .onEnter{
            drive.followTrajectorySequenceAsync(moveIntoWarehouseTwo)
            jugaad.arm.moveArmToBottomPos()
            jugaad.moveOuttakeToOpen()
            motionTimer.reset()
        }
        .transition{
            val value = distanceSensor.getDistance(DistanceUnit.INCH)
            value < 0.6 || motionTimer.seconds() > 5.0
        }

        .state(InitialDepositStates.CYCLE_DEPOSIT_TWO)
        .onEnter{
            drive.followTrajectorySequenceAsync(cycleDepositTwo)
        }
        .transition{!drive.isBusy}

        .state(InitialDepositStates.PARK_AT_END)
        .onEnter{
            drive.followTrajectorySequenceAsync(parkAtEnd)
            jugaad.arm.moveArmToBottomPos()
        }
        .transition{!drive.isBusy}

        .build()



    //Initialization
    override fun init() {
        drive = SampleMecanumDrive(hardwareMap)
        arm.init(hardwareMap)
        outtakeServo = hardwareMap.get(Servo::class.java, "Outtake") as Servo
        intakeMotor = hardwareMap.dcMotor["Intake"]
        intakeMotor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        distanceSensor = hardwareMap.get(Rev2mDistanceSensor::class.java, "distanceSensor") as Rev2mDistanceSensor

        drive.poseEstimate = startPose

        initialDepositTrajectoryTop = drive.trajectorySequenceBuilder(startPose)
            .setReversed(true)
            .splineToSplineHeading( Pose2d(5.0, 30.0, Math.toRadians(50.0)), Math.toRadians(220.0))
            .addTemporalMarker(0.2) {
                jugaad.arm.moveArmToTopPos()
            }
            .addTemporalMarker(1.5) {
                jugaad.moveOuttakeToDeposit()
            }
            .lineToSplineHeading( Pose2d(-11.0, 45.0, Math.toRadians(90.0)))
            .build()

        initialDepositTrajectoryMiddle = drive.trajectorySequenceBuilder(startPose)
            .setReversed(true)
            .splineToSplineHeading( Pose2d(0.0, 42.0, Math.toRadians(310.0)), Math.toRadians(130.0))
            .addTemporalMarker(0.2) {
                jugaad.arm.moveArmToMidPos()
            }
            .addTemporalMarker(1.5) {
                jugaad.moveOuttakeToDeposit()
            }
            .lineToSplineHeading( Pose2d(-11.0, 45.0, Math.toRadians(270.0)))
            .build()

        initialDepositTrajectoryBottom = drive.trajectorySequenceBuilder(startPose)
            .setReversed(true)
            .splineToSplineHeading( Pose2d(0.0, 42.0, Math.toRadians(310.0)), Math.toRadians(130.0))
            .addTemporalMarker(0.2) {
                jugaad.arm.moveArmToBottomPos()
            }
            .addTemporalMarker(1.5) {
                jugaad.moveOuttakeToDeposit()
            }
            .lineToSplineHeading( Pose2d(-11.0, 45.0, Math.toRadians(90.0)))
            .build()

        moveIntoWarehouseOne = drive.trajectorySequenceBuilder(depositPose)

            .setReversed(false)
            .splineToSplineHeading( Pose2d(43.0, 70.5, Math.toRadians(0.0)), Math.toRadians(0.0))
            .splineToConstantHeading(Vector2d(48.5, 70.5), Math.toRadians(0.0))
            .splineToConstantHeading(Vector2d(43.0, 70.5), Math.toRadians(0.0))
            .addTemporalMarker(1.5) {

                jugaad.intakeFreight()
            }
            .build()

        cycleDepositOne = drive.trajectorySequenceBuilder(warehouseOnePose)

            .setReversed(true)
            .splineToSplineHeading( Pose2d(-11.0, 45.0, Math.toRadians(90.0)), Math.toRadians(270.0))
            .addTemporalMarker(0.5) {
                jugaad.moveOuttakeToLock()
                jugaad.reverseIntake()
            }
            .addTemporalMarker(2.25) {
                jugaad.arm.moveArmToTopPos()
                jugaad.stopIntake()
            }
            .addTemporalMarker(3.0) {
                jugaad.moveOuttakeToDeposit()
            }
            .build()

        moveIntoWarehouseTwo = drive.trajectorySequenceBuilder(depositPose)

            .setReversed(false)
            .splineToSplineHeading(Pose2d(46.0, 70.5, Math.toRadians(0.0)), Math.toRadians(0.0))
            .build()

        cycleDepositTwo = drive.trajectorySequenceBuilder(warehouseTwoPose)

            .setReversed(true)
            .splineToSplineHeading(Pose2d(-11.0, 45.0, Math.toRadians(90.0)), Math.toRadians(270.0))
            .addTemporalMarker(0.2) {
                jugaad.reverseIntake()
            }
            .addTemporalMarker(1.0) {
                jugaad.arm.moveArmToTopPos()
                jugaad.stopIntake()
            }
            .addTemporalMarker(1.5) {
                jugaad.moveOuttakeToDeposit()
            }
            .build()

        parkAtEnd = drive.trajectorySequenceBuilder(depositPose)

            .setReversed(false)
            .splineToSplineHeading(Pose2d(40.0, 70.5, Math.toRadians(0.0)), Math.toRadians(0.0))
            .build()

        initialDepositStateMachine.start()


        jugaad = Jugaad(
            intakeMotor, outtakeServo, distanceSensor, arm,
        )

        jugaad.moveOuttakeToLock()
    }


    //Loop & Updates
    override fun loop() {
        initialDepositStateMachine.update()
        drive.update()
        arm.update()
    }
}