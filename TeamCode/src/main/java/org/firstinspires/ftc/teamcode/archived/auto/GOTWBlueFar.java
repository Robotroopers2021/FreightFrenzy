package org.firstinspires.ftc.teamcode.archived.auto;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;


@Autonomous (name= "GOTWBlueFar")
public class GOTWBlueFar extends LinearOpMode {

    SampleMecanumDrive drive;


    DcMotor DuckL,Arm;

    @Override
    public void runOpMode() throws InterruptedException {
        DuckL = hardwareMap.get(DcMotor.class, "DuckL");
        Arm = hardwareMap.dcMotor.get("Arm");
        drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(15, 56, Math.toRadians(0));
        drive.setPoseEstimate(startPose);
        TrajectorySequence traj1 = drive.trajectorySequenceBuilder(new Pose2d(9.0, 62.0, Math.toRadians(90.0)))
                .setReversed(true)
                .splineToSplineHeading(new Pose2d(3.5, 37.5, Math.toRadians(50.0)), Math.toRadians(230.0))
                .setReversed(false)
                .splineToSplineHeading(new Pose2d(20.5, 63.5, Math.toRadians(0.0)), Math.toRadians(0.0))
                .splineToSplineHeading(new Pose2d(40.5, 63.5, Math.toRadians(0.0)), Math.toRadians(0.0))
                .setReversed(true)
                .splineToSplineHeading(new Pose2d(20.5, 63.5, Math.toRadians(0.0)), Math.toRadians(180.0))
                .splineToSplineHeading(new Pose2d(3.5, 37.5, Math.toRadians(50.0)), Math.toRadians(230.0))
                .setReversed(false)
                .splineToSplineHeading(new Pose2d(20.5, 63.5, Math.toRadians(0.0)), Math.toRadians(0.0))
                .splineToSplineHeading(new Pose2d(45.5, 63.5, Math.toRadians(0.0)), Math.toRadians(0.0))
                .setReversed(true)
                .splineToSplineHeading(new Pose2d(20.5, 63.5, Math.toRadians(0.0)), Math.toRadians(180.0))
                .splineToSplineHeading(new Pose2d(3.5, 37.5, Math.toRadians(50.0)), Math.toRadians(230.0))
                .setReversed(false)
                .splineToSplineHeading(new Pose2d(20.5, 63.5, Math.toRadians(0.0)), Math.toRadians(0.0))
                .splineToSplineHeading(new Pose2d(50.5, 63.5, Math.toRadians(0.0)), Math.toRadians(0.0))
                .setReversed(true)
                .splineToSplineHeading(new Pose2d(20.5, 63.5, Math.toRadians(0.0)), Math.toRadians(180.0))
                .splineToSplineHeading(new Pose2d(3.5, 37.5, Math.toRadians(50.0)), Math.toRadians(230.0))
                .setReversed(false)
                .splineToSplineHeading(new Pose2d(20.5, 63.5, Math.toRadians(0.0)), Math.toRadians(0.0))
                .splineToSplineHeading(new Pose2d(55.5, 63.5, Math.toRadians(0.0)), Math.toRadians(0.0))
                .setReversed(true)
                .splineToSplineHeading(new Pose2d(20.5, 63.5, Math.toRadians(0.0)), Math.toRadians(180.0))
                .splineToSplineHeading(new Pose2d(3.5, 37.5, Math.toRadians(50.0)), Math.toRadians(230.0))
                .setReversed(false)
                .splineToSplineHeading(new Pose2d(20.5, 63.5, Math.toRadians(0.0)), Math.toRadians(0.0))
                .splineToSplineHeading(new Pose2d(45.5, 63.5, Math.toRadians(0.0)), Math.toRadians(0.0))
                .build();
        waitForStart();
        drive.followTrajectorySequence(traj1);
        while(!isStopRequested() && drive.isBusy());
    }
}




