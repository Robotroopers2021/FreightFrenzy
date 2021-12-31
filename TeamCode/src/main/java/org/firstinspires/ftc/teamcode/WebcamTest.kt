package org.firstinspires.ftc.teamcode


import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.acmerobotics.roadrunner.trajectory.Trajectory
import com.qualcomm.hardware.rev.Rev2mDistanceSensor
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive
import org.firstinspires.ftc.teamcode.stateMachine.StateMachineBuilder
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence
import org.firstinspires.ftc.teamcode.util.math.Pose
import org.firstinspires.ftc.teamcode.vision.AllianceSide
import org.firstinspires.ftc.teamcode.vision.Globals
import org.firstinspires.ftc.teamcode.vision.Webcam

@Autonomous(preselectTeleOp = "CompTeleOp")
class WebcamTest : OpMode() {
    //Start Pose Values
    private val startX = 11.0
    private val startY = 61.25
    private val startAngle = Math.toRadians(90.0)
    //Deposit Pose Values
    private val depositX = 11.0
    private val depositY = 45.0
    private val depositAngle = Math.toRadians(270.0)
    // Warehouse One Pose Values
    private val warehouseOneX = 43.0
    private val warehouseOneY = 64.5
    private val warehouseOneAngle = Math.toRadians(0.0)
    //Warehouse Two Pose Values
    private val warehouseTwoX = 46.0
    private val warehouseTwoY = 64.5
    private val warehouseTwoAngle = Math.toRadians(0.0)
    //Arm.kt and Jugaad.kt instances
    private val arm = Arm()
    private val jugaad = Jugaad(
        intakeMotor, outtakeServo, distanceSensor, arm,
    )

    //Start Pose
    private val startPose = if(Globals.ALLIANCE_SIDE == AllianceSide.BLUE) {
        Pose2d(startX, startY, startAngle)
    } else {
        Pose2d(startX, -startY, -startAngle)
    }
    //Deposit Pose
    private val depositPose = if(Globals.ALLIANCE_SIDE == AllianceSide.BLUE) {
        Pose2d(depositX, depositY, depositAngle)
    } else {
        Pose2d(depositX, -depositY, -depositAngle)
    }
    //Warehouse One Pose
    private val warehouseOnePose = if (Globals.ALLIANCE_SIDE == AllianceSide.BLUE) {
        Pose2d(warehouseOneX, warehouseOneY, warehouseOneAngle)
    } else {
        Pose2d(warehouseOneX, -warehouseOneY, -warehouseOneAngle)
    }
    //Warehouse Two Pose
    private val warehouseTwoPose = if (Globals.ALLIANCE_SIDE == AllianceSide.BLUE) {
        Pose2d(warehouseTwoX, warehouseTwoY, warehouseTwoAngle)
    } else {
        Pose2d(warehouseTwoX, -warehouseTwoY, -warehouseTwoAngle)
    }
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
        .loop {
            when(Globals.CUP_LOCATION) {
                Webcam.CupStates.RIGHT -> drive.followTrajectorySequenceAsync(initialDepositTrajectoryTop)
                Webcam.CupStates.MIDDLE -> drive.followTrajectorySequenceAsync(initialDepositTrajectoryMiddle)
                Webcam.CupStates.LEFT -> drive.followTrajectorySequenceAsync(initialDepositTrajectoryBottom)
            }
        }
        .transition{!drive.isBusy}

        .state(InitialDepositStates.MOVE_INTO_WAREHOUSE_ONE)
        .onEnter{
            drive.followTrajectorySequenceAsync(moveIntoWarehouseOne)
            jugaad.arm.moveArmToBottomPos()
            jugaad.intakeFreight()
        }
        .transition{
            val value = distanceSensor.getDistance(DistanceUnit.INCH)
            value < 0.6 || motionTimer.seconds() > 5.0
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
            jugaad.intakeFreight()
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
        jugaad.moveOuttakeToLock()
        intakeMotor = hardwareMap.dcMotor["Intake"]
        intakeMotor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        distanceSensor = hardwareMap.get(Rev2mDistanceSensor::class.java, "distanceSensor") as Rev2mDistanceSensor

        drive.poseEstimate = startPose

        initialDepositTrajectoryTop = drive.trajectorySequenceBuilder(startPose)
            .setReversed(true)
            .splineToSplineHeading( Pose2d(0.0, 42.0, Math.toRadians(310.0)), Math.toRadians(130.0))
            .addTemporalMarker(0.2) {
                jugaad.arm.moveArmToTopPos()
            }
            .addTemporalMarker(1.5) {
                jugaad.moveOuttakeToOpen()
            }
            .lineToSplineHeading( Pose2d(-11.0, 45.0, Math.toRadians(270.0)))
            .build()

        initialDepositTrajectoryMiddle = drive.trajectorySequenceBuilder(startPose)
            .setReversed(true)
            .splineToSplineHeading( Pose2d(0.0, 42.0, Math.toRadians(310.0)), Math.toRadians(130.0))
            .addTemporalMarker(0.2) {
                jugaad.arm.moveArmToMidPos()
            }
            .addTemporalMarker(1.5) {
                jugaad.moveOuttakeToOpen()
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
                jugaad.moveOuttakeToOpen()
            }
            .lineToSplineHeading( Pose2d(-11.0, 45.0, Math.toRadians(270.0)))
            .build()

        moveIntoWarehouseOne = drive.trajectorySequenceBuilder(depositPose)

            .setReversed(false)
            .splineToSplineHeading( Pose2d(43.0, 64.5, Math.toRadians(0.0)), Math.toRadians(0.0))
            .build()

        cycleDepositOne = drive.trajectorySequenceBuilder(warehouseOnePose)

            .setReversed(true)
            .splineToSplineHeading( Pose2d(-11.0, -45.0, Math.toRadians(270.0)), Math.toRadians(90.0))
            .addTemporalMarker(6.5) {
                jugaad.reverseIntake()
            }
            .addTemporalMarker(8.2) {
                jugaad.arm.moveArmToTopPos()
                jugaad.stopIntake()
            }
            .addTemporalMarker(9.5,) {
                jugaad.moveOuttakeToOpen()
            }
            .build()

        moveIntoWarehouseTwo = drive.trajectorySequenceBuilder(depositPose)

            .setReversed(false)
            .splineToSplineHeading(Pose2d(46.0, -64.5, Math.toRadians(0.0)), Math.toRadians(0.0))
            .build()

        cycleDepositTwo = drive.trajectorySequenceBuilder(warehouseTwoPose)

            .setReversed(true)
            .splineToSplineHeading(Pose2d(-11.0, -45.0, Math.toRadians(270.0)), Math.toRadians(90.0))
            .addTemporalMarker(13.5) {
                jugaad.reverseIntake()
            }
            .addTemporalMarker(14.2) {
                jugaad.arm.moveArmToTopPos()
                jugaad.stopIntake()
            }
            .addTemporalMarker(15.5,) {
                jugaad.moveOuttakeToOpen()
            }
            .build()

        parkAtEnd = drive.trajectorySequenceBuilder(depositPose)

            .setReversed(false)
            .splineToSplineHeading(Pose2d(40.0, -64.5, Math.toRadians(0.0)), Math.toRadians(0.0))
            .build()

        initialDepositStateMachine.start()
    }


    //Loop & Updates
    override fun loop() {
        initialDepositStateMachine.update()
        drive.update()
        arm.update()
    }
}