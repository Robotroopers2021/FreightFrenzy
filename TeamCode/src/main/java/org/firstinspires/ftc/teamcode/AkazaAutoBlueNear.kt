package org.firstinspires.ftc.teamcode

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.qualcomm.hardware.rev.Rev2mDistanceSensor
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.robotcore.external.StateMachine
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive
import org.firstinspires.ftc.teamcode.stateMachine.StateMachineBuilder
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence
import org.firstinspires.ftc.teamcode.vision.WebcamTest

@Autonomous(preselectTeleOp = "CompTeleOp")
class AkazaAutoBlueNear : OpMode()  {

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
        outtakeServo.position = 0.80
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
        .onEnter{drive.followTrajectorySequenceAsync(DuckSpinnerTraj)}
        .transition{!drive.isBusy}

        .build()

    override fun init() {
        DuckL = hardwareMap.get(DcMotor::class.java, "DuckL")
        drive = SampleMecanumDrive(hardwareMap)
        outtakeServo = hardwareMap.get(Servo::class.java, "Outtake") as Servo
        outtakeServo.position = 0.80
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
            DuckL.setPower( 0.75 )
        }
        .waitSeconds( 2.2 )
            .addTemporalMarker {
            DuckL.setPower(0.0)
        }
        .lineToSplineHeading( Pose2d(-61.0, 33.0, Math.toRadians(0.0)))
            .build()


        drive.poseEstimate = Pose2d(-36.0, 60.0, Math.toRadians(90.0))

        duckSpinnerStateMachine.start()
    }

    override fun loop() {
        duckSpinnerStateMachine.update()
        drive.update()
        arm.update()
    }
}