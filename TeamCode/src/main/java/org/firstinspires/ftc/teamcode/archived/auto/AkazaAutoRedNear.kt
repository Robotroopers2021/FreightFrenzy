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
import org.firstinspires.ftc.teamcode.vision.WebcamTest

@Autonomous(preselectTeleOp = "AkazaRedOp")
class AkazaAutoRedNear : OpMode()  {

    private var motionTimer = ElapsedTime()

    private lateinit var outtakeServo: Servo

    private lateinit var intakeMotor: DcMotor

    private val arm = Arm()

    private lateinit var  DuckL : DcMotor

    private val webcam = WebcamTest()

    private lateinit var InitialDepositTrajTop : TrajectorySequence

    private lateinit var InitialDepositTrajMiddle : TrajectorySequence

    private lateinit var InitialDepositTrajBottom : TrajectorySequence

    private lateinit var DuckSpinnerTraj : TrajectorySequence

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
                Pipeline.CupStates.RIGHT -> drive.followTrajectorySequenceAsync(
                    InitialDepositTrajBottom
                )
                Pipeline.CupStates.CENTER -> drive.followTrajectorySequenceAsync(
                    InitialDepositTrajMiddle
                )
                Pipeline.CupStates.LEFT -> drive.followTrajectorySequenceAsync(
                    InitialDepositTrajTop
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

        DuckSpinnerTraj = drive.trajectorySequenceBuilder(Pose2d (-36.0, 60.0, Math.toRadians(90.0)))
            .setReversed(true)
            .splineToSplineHeading( Pose2d(-11.0, 40.0, Math.toRadians(110.0)), Math.toRadians(325.0))
            .waitSeconds(1.0)
            .addTemporalMarker(1.5) {
                arm.moveArmToTopPos()
            }
            .addTemporalMarker(2.1) {
                moveOuttakeToOut()
            }
            .addTemporalMarker(3.0) {
                arm.moveArmToBottomPos()
                moveOuttakeToOpen()
            }
            .setReversed(false)
            .lineToConstantHeading( Vector2d(-59.0, 57.0))
            .addTemporalMarker {
                DuckL.setPower( 0.25 )
            }
            .waitSeconds( 6.0 )
            .addTemporalMarker {
                DuckL.setPower(0.0)
            }
            .lineToSplineHeading( Pose2d(-61.0, 33.0, Math.toRadians(0.0)))
            .build()


        drive.poseEstimate = Pose2d(-36.0, 60.0, Math.toRadians(90.0))

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