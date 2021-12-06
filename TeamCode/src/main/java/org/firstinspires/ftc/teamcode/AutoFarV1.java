package org.firstinspires.ftc.teamcode;

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


@Autonomous (name= "AutoFarV1")
public class AutoFarV1 extends LinearOpMode {

    SampleMecanumDrive drive;

    double kp = 0.015;
    double ki = 0;
    double kd = 0;
    double targetPosition = 415;
    double min = -0.5;
    double max = 0.5;

    DcMotor DuckL,Arm;
    PIDFController Controller = new PIDFController(new PIDCoefficients(kp,ki,kd));


    @Override
    public void runOpMode() throws InterruptedException {
        DuckL = hardwareMap.get(DcMotor.class, "DuckL");
        Arm = hardwareMap.dcMotor.get("Arm");
        double output = Controller.update(Arm.getCurrentPosition());
        Arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(15, 56, Math.toRadians(90));
        drive.setPoseEstimate(startPose);
        TrajectorySequence traj1 = drive.trajectorySequenceBuilder(new Pose2d(15, 56, Math.toRadians(90)))
                .addTemporalMarker( () -> {
                    Arm.setPower(Range.clip(output,min,max));
                    Controller.setTargetPosition(targetPosition);
                } )
                .waitSeconds(2)
                .addTemporalMarker( ( ) -> {
                    Arm.setPower(Range.clip(output,min,max));
                    Controller.setTargetPosition(25);
                } )
                .lineToConstantHeading(new Vector2d(-11, 35))
                .lineToConstantHeading(new Vector2d(-59, 31))
                .lineToConstantHeading(new Vector2d(-59, 50))
                .addTemporalMarker( ( ) -> {
                    DuckL.setPower( -0.75 );
                } )
                .waitSeconds( 2.2 )
                .addTemporalMarker( ( ) -> {
                    DuckL.setPower(0);
                } )
                .lineToConstantHeading(new Vector2d(-59, 40))
                .lineToSplineHeading(new Pose2d(-8, 31, Math.toRadians(0)))
                .lineToConstantHeading(new Vector2d(58, 40))
                .addTemporalMarker( ( ) -> {
                    Arm.setPower(Range.clip(output,min,max));
                    Controller.setTargetPosition(0);
                } )
                .build();
        waitForStart();
        drive.followTrajectorySequence(traj1);
        while(!isStopRequested() && drive.isBusy());
    }
}




