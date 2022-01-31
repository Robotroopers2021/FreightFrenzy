package org.firstinspires.ftc.teamcode.archived.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous
public class RedNearScrim extends LinearOpMode {
    SampleMecanumDrive drive;

    DcMotor DuckL;

    @Override
    public void runOpMode() throws InterruptedException {
        DuckL = hardwareMap.get(DcMotor.class, "DuckL");
        drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(-36, -60, Math.toRadians(180));
        drive.setPoseEstimate(startPose);
        TrajectorySequence traj = drive.trajectorySequenceBuilder(new Pose2d(-36, -60, Math.toRadians(180)))
                .lineToConstantHeading(new Vector2d(-57, -55))
                .addTemporalMarker( ( ) -> {
                    DuckL.setPower( 0.75 );
                } )
                .waitSeconds( 2.2 )
                .addTemporalMarker( ( ) -> {
                    DuckL.setPower(0);
                } )
                .lineToSplineHeading(new Pose2d(-63, -28, Math.toRadians(0)))
                .build();

        waitForStart();
        drive.followTrajectorySequence(traj);
        while (!isStopRequested() && drive.isBusy()) ;
    }
}
