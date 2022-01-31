package org.firstinspires.ftc.teamcode.testing

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
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive
import org.firstinspires.ftc.teamcode.stateMachine.StateMachineBuilder
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence
import org.firstinspires.ftc.teamcode.vision.*

@Autonomous(preselectTeleOp = "AkazaRedOp")
class AutoTestingRed : OpMode() {

    private var startPose = Pose2d(11.0, -57.25, Math.toRadians(270.0))

    private var motionTimer = ElapsedTime()

    private var value = 0.0

    private lateinit var outtakeServo: Servo

    private lateinit var intakeMotor: DcMotor

    private lateinit var distanceSensor : Rev2mDistanceSensor

    private val arm = Arm()

    private val webcam = WebcamRed()

    private lateinit var InitialDepositTrajTop : TrajectorySequence

    private lateinit var InitialDepositTrajMiddle : TrajectorySequence

    private lateinit var InitialDepositTrajBottom : TrajectorySequence

    private lateinit var CycleOneWarehouseTraj : TrajectorySequence

    private lateinit var CycleOneBottomWarehouseTraj : TrajectorySequence

    private lateinit var CycleOneDepsoitTraj : TrajectorySequence

    private lateinit var CycleTwoWarehouseTraj : TrajectorySequence

    private lateinit var CycleTwoDepsoitTraj : TrajectorySequence

    private lateinit var ParkAtEnd :TrajectorySequence



    private fun moveOuttakeToOut(){
        outtakeServo.position = 0.60

    }

    private fun moveOuttakeToLock(){
        outtakeServo.position = 0.78
    }

    private fun moveOuttakeToOpen(){
        outtakeServo.position = 0.92

    }

    private fun intakeFreight(){
        intakeMotor.power = -1.0
    }

    private fun stopIntake(){
        intakeMotor.power = 0.0
    }

    private fun getFreightOut(){
        intakeMotor.power = 1.0
    }



    lateinit var drive: SampleMecanumDrive


    private enum class InitialDepositStates {
        INITIAL_DEPOSIT,
        CYCLE_ONE_WAREHOUSE,
        CYCLE_ONE_DEPOSIT,
        CYCLE_TWO_WAREHOUSE,
        CYCLE_TWO_DEPOSIT,
        PARK_AT_END,

    }


    private val initialDepositStateMachine = StateMachineBuilder<InitialDepositStates>()
        .state(InitialDepositStates.INITIAL_DEPOSIT)
        .onEnter {
            when (webcam.pipeline.cupState) {
                PipelineRed.CupStates.RIGHT -> drive.followTrajectorySequenceAsync(InitialDepositTrajBottom)
                PipelineRed.CupStates.CENTER -> drive.followTrajectorySequenceAsync(InitialDepositTrajMiddle)
                PipelineRed.CupStates.LEFT -> drive.followTrajectorySequenceAsync(InitialDepositTrajTop)
            }
        }
        .onExit (motionTimer::reset)
        .transition{!drive.isBusy}
        .state(InitialDepositStates.CYCLE_ONE_WAREHOUSE)
        .onEnter{
            when (webcam.pipeline.cupState) {
                PipelineRed.CupStates.RIGHT -> drive.followTrajectorySequenceAsync(CycleOneWarehouseTraj)
                PipelineRed.CupStates.CENTER -> drive.followTrajectorySequenceAsync(CycleOneWarehouseTraj)
                PipelineRed.CupStates.LEFT -> drive.followTrajectorySequenceAsync(CycleOneBottomWarehouseTraj)
            }
        }
        .transition { !drive.isBusy || (value < 3 && motionTimer.seconds() > 3.0)}
        .state(InitialDepositStates.CYCLE_ONE_DEPOSIT)
        .onEnter{
            drive.followTrajectorySequenceAsync(CycleOneDepsoitTraj)
        }
        .onExit (motionTimer::reset)
        .transition{!drive.isBusy}
        .state(InitialDepositStates.CYCLE_TWO_WAREHOUSE)
        .onEnter{
            drive.followTrajectorySequenceAsync(CycleTwoWarehouseTraj)
            arm.moveArmToBottomPos()
        }
        .transition { !drive.isBusy || (value < 3 && motionTimer.seconds() > 2.0)}
        .state(InitialDepositStates.CYCLE_TWO_DEPOSIT)
        .onEnter{
            drive.followTrajectorySequenceAsync(CycleTwoDepsoitTraj)
            stopIntake()
        }
        .transition{!drive.isBusy}
        .onExit (motionTimer::reset)
        .state(InitialDepositStates.PARK_AT_END)
        .onEnter{
            drive.followTrajectorySequenceAsync(ParkAtEnd)
            arm.moveArmToBottomPos()
        }
        .transition{!drive.isBusy}
        .build()




    override fun init() {
        webcam.init(hardwareMap)
        drive = SampleMecanumDrive(hardwareMap)
        arm.init(hardwareMap)
        outtakeServo = hardwareMap.get(Servo::class.java, "Outtake") as Servo
        outtakeServo.position = 0.78
        intakeMotor = hardwareMap.dcMotor["Intake"]
        intakeMotor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        distanceSensor = hardwareMap.get(Rev2mDistanceSensor::class.java, "distanceSensor") as Rev2mDistanceSensor

        InitialDepositTrajTop = drive.trajectorySequenceBuilder(startPose)
            .setReversed(true)
            .splineToSplineHeading( Pose2d(3.0, -30.0, Math.toRadians(310.0)), Math.toRadians(140.0))
            .addTemporalMarker(0.2) {
                arm.moveArmToTopPos()
            }
            .addTemporalMarker(1.5) {
                moveOuttakeToOut()
            }
            .lineToSplineHeading( Pose2d(-11.0, -45.0, Math.toRadians(270.0)))
            .build()

        InitialDepositTrajMiddle = drive.trajectorySequenceBuilder(startPose)
            .setReversed(true)
            .splineToSplineHeading( Pose2d(7.5, -37.5, Math.toRadians(310.0)), Math.toRadians(140.0))
            .addTemporalMarker(0.2) {
                arm.moveArmToMidPos()
            }
            .addTemporalMarker(1.5) {
                moveOuttakeToOut()
            }
            .addTemporalMarker(2.25) {
                moveOuttakeToOpen()
            }
            .lineToSplineHeading( Pose2d(-11.0, -45.0, Math.toRadians(270.0)))
            .build()

        InitialDepositTrajBottom = drive.trajectorySequenceBuilder(startPose)
            .setReversed(true)
            .splineToSplineHeading( Pose2d(8.0, -41.0, Math.toRadians(310.0)), Math.toRadians(140.0))
            .addTemporalMarker(0.2) {
                arm.autoBottomPos()
            }
            .addTemporalMarker(1.5) {
                moveOuttakeToOut()
            }
            .addTemporalMarker(2.25) {
                moveOuttakeToOpen()
            }
            .lineToSplineHeading( Pose2d(-11.0, -48.0, Math.toRadians(270.0)))
            .build()

        CycleOneWarehouseTraj = drive.trajectorySequenceBuilder(Pose2d(-11.0, -45.0 , Math.toRadians(270.0)))
            .setReversed(false)
            .splineToSplineHeading(Pose2d(34.0, -65.75, Math.toRadians(0.0)), Math.toRadians(0.0))
            .splineToConstantHeading(Vector2d(49.5, -67.75), Math.toRadians(0.0))
            .addTemporalMarker(0.1) {
                moveOuttakeToOpen()
            }
            .addTemporalMarker( 0.75) {
                arm.moveArmToBottomPos()
            }
            .addTemporalMarker(2.5) {
                intakeFreight()
            }
            .waitSeconds(0.5)
            .build()

        CycleOneBottomWarehouseTraj = drive.trajectorySequenceBuilder(Pose2d(-11.0, -48.0 , Math.toRadians(270.0)))
            .setReversed(false)
            .splineToSplineHeading(Pose2d(36.0, -65.75, Math.toRadians(0.0)), Math.toRadians(0.0))
            .splineToConstantHeading(Vector2d(49.5, -67.75), Math.toRadians(0.0))
            .addTemporalMarker(0.1) {
                moveOuttakeToOpen()
            }
            .addTemporalMarker( 0.75) {
                arm.moveArmToBottomPos()
            }
            .addTemporalMarker(2.5) {
                intakeFreight()
            }
            .waitSeconds(0.5)
            .build()

        CycleOneDepsoitTraj = drive.trajectorySequenceBuilder(Pose2d(49.5, -67.75, Math.toRadians(0.0)))
            .setReversed(true)
            .addTemporalMarker(0.1) {
                getFreightOut()
            }
            .addTemporalMarker(1.1) {
                moveOuttakeToLock()
                stopIntake()
            }
            .addTemporalMarker(1.5) {
                arm.moveArmToTopPos()
            }
            .addTemporalMarker(2.75) {
                moveOuttakeToOut()
            }
            .splineToConstantHeading( Vector2d(40.0, -67.75), Math.toRadians(180.0))
            .splineToSplineHeading( Pose2d(-11.0, -45.0 , Math.toRadians(270.0)), Math.toRadians(90.0))
            .splineToConstantHeading( Vector2d(-11.0, -41.0 ), Math.toRadians(90.0))
            .setReversed(false)
            .splineToConstantHeading( Vector2d(-11.0, -45.0), Math.toRadians(270.0))
            .build()

        CycleTwoWarehouseTraj = drive.trajectorySequenceBuilder( Pose2d(-11.0, -45.0 , Math.toRadians(270.0)))
            .setReversed(false)
            .addTemporalMarker(0.1) {
                moveOuttakeToOpen()
            }
            .addTemporalMarker(1.5) {
                intakeFreight()
            }
            .splineToSplineHeading(Pose2d(40.0, -69.75, Math.toRadians(0.0)), Math.toRadians(0.0))
            .splineToConstantHeading(Vector2d(52.0, -67.75), Math.toRadians(10.0))
            .waitSeconds(1.0)
            .build()

        CycleTwoDepsoitTraj = drive.trajectorySequenceBuilder(Pose2d(52.0, -69.75, Math.toRadians(0.0)))
            .setReversed(true)
            .addTemporalMarker(0.1) {
                moveOuttakeToLock()
                getFreightOut()
            }
            .addTemporalMarker(1.1) {
                moveOuttakeToLock()
                stopIntake()
            }
            .addTemporalMarker(1.5) {
                arm.moveArmToTopPos()
            }
            .addTemporalMarker(2.75) {
                moveOuttakeToOut()
            }
            .splineToConstantHeading( Vector2d(40.0, -69.75), Math.toRadians(180.0))
            .splineToSplineHeading( Pose2d(-11.0, -45.0 , Math.toRadians(270.0)), Math.toRadians(90.0))
            .build()

        ParkAtEnd = drive.trajectorySequenceBuilder( Pose2d(-11.0, -45.0 , Math.toRadians(270.0)))
            .setReversed(false)
            .splineToSplineHeading(Pose2d(40.0, -70.75, Math.toRadians(0.0)), Math.toRadians(0.0))
            .addTemporalMarker(0.1) {
                moveOuttakeToOpen()
            }
            .waitSeconds(1.0)
            .build()


        drive.poseEstimate = startPose



    }

    override fun init_loop() {
        super.init_loop()
        webcam.update()
        telemetry.addData("Cup State", webcam.pipeline.cupState)
        telemetry.addData("Left Total", webcam.pipeline.LeftTotal)
        telemetry.addData("Center Total", webcam.pipeline.CenterTotal)
        telemetry.addData("Right Total", webcam.pipeline.RightTotal)
        telemetry.update()

    }

    override fun start() {
        super.start()
        webcam.reset()
        initialDepositStateMachine.start()

    }

    override fun loop() {
        initialDepositStateMachine.update()
        drive.update()
        arm.update()
        value = distanceSensor.getDistance(DistanceUnit.INCH)
        telemetry.addData("Cup State", webcam.pipeline.cupState)
        telemetry.update()
    }
}