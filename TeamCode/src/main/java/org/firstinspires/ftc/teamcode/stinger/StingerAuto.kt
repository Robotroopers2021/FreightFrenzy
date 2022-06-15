package org.firstinspires.ftc.teamcode.stinger

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.qualcomm.hardware.rev.Rev2mDistanceSensor
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap
import org.firstinspires.ftc.teamcode.drive.DriveConstants
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive
import org.firstinspires.ftc.teamcode.stateMachine.StateMachineBuilder
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence
import org.firstinspires.ftc.teamcode.vision.Pipeline

@Autonomous
class StingerAuto : OpMode() {
    private lateinit var trajectory : TrajectorySequence

    val startPose = Pose2d(6.25, 63.0, Math.toRadians(90.0))

    private enum class InitialDepositStates {
        TRAJECTORY
    }

    lateinit var drive: SampleMecanumDrive


    private val initialDepositStateMachine = StateMachineBuilder<InitialDepositStates>()
        .state(InitialDepositStates.TRAJECTORY)
        .onEnter{
            drive.followTrajectorySequence(trajectory)
        }
        .build()




    override fun init() {
        drive = SampleMecanumDrive(hardwareMap)

        trajectory = drive.trajectorySequenceBuilder(startPose)
            .setReversed(true)
            //initial deposit
            .lineToLinearHeading( Pose2d(-5.0, 47.0, -Math.toRadians(295.0)),
                SampleMecanumDrive.getVelocityConstraint(35.0, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
            .setReversed(false)
            //first half of warehouse one
            .splineToSplineHeading( Pose2d(17.0, 64.0, Math.toRadians(360.0)), -Math.toRadians(360.0))
            //second half of warehouse one
            .splineToConstantHeading( Vector2d(42.0, 64.0), Math.toRadians(0.0),
                SampleMecanumDrive.getVelocityConstraint(40.0, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                SampleMecanumDrive.getAccelerationConstraint(20.0))
            .setReversed(true)
            //exiting warehouse one
            .splineToConstantHeading( Vector2d(17.0, 64.0), -Math.toRadians(180.0),
                SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
            //deposit one
            .splineTo( Vector2d(-5.0, 44.2), Math.toRadians(260.0),
                SampleMecanumDrive.getVelocityConstraint(55.0, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
            .setReversed(false)
            //first half of warehouse two
            .splineToSplineHeading( Pose2d(17.0, 64.0, Math.toRadians(360.0)), Math.toRadians(360.0),
                SampleMecanumDrive.getVelocityConstraint(45.0, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
            )
            //second half of warehouse two
            .splineToConstantHeading( Vector2d(48.0, 64.0), Math.toRadians(0.0),
                SampleMecanumDrive.getVelocityConstraint(40.0, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                SampleMecanumDrive.getAccelerationConstraint(20.0))
            .setReversed(true)
            //exiting warehouse two
            .splineToConstantHeading( Vector2d(17.0, 64.0), -Math.toRadians(180.0),
                SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
            //deposit two
            .splineTo( Vector2d(-5.0, 44.2), Math.toRadians(260.0),
                SampleMecanumDrive.getVelocityConstraint(55.0, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
            .setReversed(false)
            .build()



        drive.poseEstimate = startPose



    }

    override fun loop() {
        initialDepositStateMachine.update()
        drive.update()
    }
}