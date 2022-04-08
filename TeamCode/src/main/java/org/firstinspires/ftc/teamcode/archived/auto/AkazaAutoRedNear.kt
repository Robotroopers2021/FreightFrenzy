package org.firstinspires.ftc.teamcode.archived.auto

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.archived.teleop.Arm
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive
import org.firstinspires.ftc.teamcode.stateMachine.StateMachineBuilder
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence
import org.firstinspires.ftc.teamcode.vision.Pipeline
import org.firstinspires.ftc.teamcode.vision.PipelineRed
import org.firstinspires.ftc.teamcode.vision.WebcamRed
import org.firstinspires.ftc.teamcode.vision.WebcamTest

@Autonomous(preselectTeleOp = "AkazaBlueOp")
class AkazaAutoRedNear : OpMode()  {

    private var motionTimer = ElapsedTime()

    private lateinit var outtakeServo: Servo

    private lateinit var intakeMotor: DcMotor

    private val arm = Arm()

    private lateinit var  DuckL : DcMotor

    private val webcam = WebcamRed()

    private lateinit var InitialDepositTrajTop : TrajectorySequence

    private lateinit var InitialDepositTrajMiddle : TrajectorySequence

    private lateinit var InitialDepositTrajBottom : TrajectorySequence

    private fun moveOuttakeToOut(){
        outtakeServo.position = 0.60

    }

    private fun moveOuttakeToLock(){
        outtakeServo.position = 0.78
    }

    private fun moveOuttakeToOpen(){
        outtakeServo.position = 0.90

    }

    lateinit var drive: SampleMecanumDrive

    private enum class DuckSpinnerStates {
        DUCK_SPINNER
    }

    private val duckSpinnerStateMachine = StateMachineBuilder<DuckSpinnerStates>()
        .state(DuckSpinnerStates.DUCK_SPINNER)
        .onEnter {
            when (webcam.pipeline.cupState) {
                PipelineRed.CupStates.LEFT -> drive.followTrajectorySequenceAsync(
                    InitialDepositTrajTop
                )
                PipelineRed.CupStates.CENTER -> drive.followTrajectorySequenceAsync(
                    InitialDepositTrajMiddle
                )
                PipelineRed.CupStates.RIGHT -> drive.followTrajectorySequenceAsync(
                    InitialDepositTrajBottom
                )
            }
        }
        .transition{!drive.isBusy}

        .build()

    override fun init() {
        webcam.init(hardwareMap)
        DuckL = hardwareMap.get(DcMotor::class.java, "DuckL")
        drive = SampleMecanumDrive(hardwareMap)
        outtakeServo = hardwareMap.get(Servo::class.java, "Outtake") as Servo
        outtakeServo.position = 0.78
        arm.init(hardwareMap)

        InitialDepositTrajTop = drive.trajectorySequenceBuilder(Pose2d (-38.0, -62.0, Math.toRadians(270.0)))
            .setReversed(true)
            .splineToSplineHeading( Pose2d(-27.5, -37.0, Math.toRadians(220.0)), Math.toRadians(55.0))
            .waitSeconds(1.0)
            .addTemporalMarker(1.5) {
                arm.moveArmToTopPos()
            }
            .addTemporalMarker(2.1) {
                moveOuttakeToOut()
            }
            .addTemporalMarker(2.5) {
                arm.moveArmToBottomPos()
                moveOuttakeToOpen()
            }
            .setReversed(false)
            .splineToSplineHeading( Pose2d(-60.0, -58.5, Math.toRadians(205.0)), Math.toRadians(205.0))
            .addTemporalMarker {
                DuckL.setPower( -0.25 )
            }
            .waitSeconds( 6.0 )
            .addTemporalMarker {
                DuckL.setPower(0.0)
            }
            .lineToSplineHeading( Pose2d(-63.0, -36.0, Math.toRadians(0.0)))
            .build()

        InitialDepositTrajMiddle = drive.trajectorySequenceBuilder(Pose2d (-38.0, -62.0, Math.toRadians(270.0)))
            .setReversed(true)
            .splineToSplineHeading( Pose2d(-30.0, -37.0, Math.toRadians(220.0)), Math.toRadians(55.0))
            .addTemporalMarker(0.1) {
                arm.moveArmToMidPos()
            }
            .addTemporalMarker(1.5) {
                moveOuttakeToOut()
            }
            .addTemporalMarker(2.5) {
                arm.moveArmToBottomPos()
                moveOuttakeToOpen()
            }
            .setReversed(false)
            .splineToSplineHeading( Pose2d(-60.0, -58.5, Math.toRadians(205.0)), Math.toRadians(205.0))            .addTemporalMarker {
                DuckL.setPower( -0.25 )
            }
            .waitSeconds( 6.0 )
            .addTemporalMarker {
                DuckL.setPower(0.0)
            }
            .lineToSplineHeading( Pose2d(-63.0, -36.0, Math.toRadians(0.0)))
            .build()

        InitialDepositTrajBottom = drive.trajectorySequenceBuilder(Pose2d (-38.0, -62.0, Math.toRadians(270.0)))
            .setReversed(true)
            .splineToSplineHeading( Pose2d(-30.0, -37.0, Math.toRadians(220.0)), Math.toRadians(55.0))
            .addTemporalMarker(0.1) {
                arm.autoBottomPos()
            }
            .addTemporalMarker(1.5) {
                moveOuttakeToOut()
            }
            .addTemporalMarker(2.5) {
                arm.moveArmToBottomPos()
                moveOuttakeToOpen()
            }
            .setReversed(false)
            .splineToSplineHeading( Pose2d(-60.0, -58.5, Math.toRadians(205.0)), Math.toRadians(205.0))            .addTemporalMarker {
                DuckL.setPower( -0.25 )
            }
            .waitSeconds( 6.0 )
            .addTemporalMarker {
                DuckL.setPower(0.0)
            }
            .lineToSplineHeading( Pose2d(-63.0, -36.0, Math.toRadians(0.0)))
            .build()


        drive.poseEstimate = Pose2d(-38.0, -62.0, Math.toRadians(270.0))

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
        duckSpinnerStateMachine.start()
    }

    override fun loop() {
        duckSpinnerStateMachine.update()
        drive.update()
        arm.update()
        telemetry.addData("Cup State", webcam.pipeline.cupState)
        telemetry.update()
    }
}