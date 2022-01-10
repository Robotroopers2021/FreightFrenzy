package org.firstinspires.ftc.teamcode.archived.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous
public class AutoFarRedVer1 extends LinearOpMode {

    SampleMecanumDrive drive;

    DcMotor DuckL;

    @Override
    public void runOpMode() throws InterruptedException {
        DuckL = hardwareMap.get(DcMotor.class, "DuckL");
        drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(-36, -60, Math.toRadians(270));
        drive.setPoseEstimate(startPose);
        TrajectorySequence traj = drive.trajectorySequenceBuilder(new Pose2d(15, -56, Math.toRadians(270)))
                .lineToConstantHeading(new Vector2d(-11, -35))
                .lineToSplineHeading(new Pose2d(-50, -59, Math.toRadians(180)))
                .addTemporalMarker(() -> {
                    DuckL.setPower(-0.75);
                })
                .waitSeconds(2.2)
                .addTemporalMarker(() -> {
                    DuckL.setPower(0);
                })
                .lineToConstantHeading(new Vector2d(-59, -40))
                .lineToSplineHeading(new Pose2d(-8, -40, Math.toRadians(0)))
                .lineToConstantHeading(new Vector2d(58, -40))
                .build();

        waitForStart();
        drive.followTrajectorySequence(traj);
        while (!isStopRequested() && drive.isBusy()) ;
    }
}
