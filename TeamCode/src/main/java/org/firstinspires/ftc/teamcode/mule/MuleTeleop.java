package org.firstinspires.ftc.teamcode.mule;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;

@TeleOp(name = "MuleTeleop")
public class MuleTeleop extends OpMode {

    DcMotor FR = null;
    DcMotor FL = null;
    DcMotor BR = null;
    DcMotor BL = null;
    DcMotor Arm = null;


    DistanceSensor distanceSensor;

    double drive;
    double strafe;
    double rotate;

    double FLPower;
    double FRPower;
    double BLPower;
    double BRPower;

    double ArmPower;

    @Override
    public void init() {
        //Connect Motor
        FL = hardwareMap.get(DcMotor.class, "FL");
        FR = hardwareMap.get(DcMotor.class, "FR");
        BL = hardwareMap.get(DcMotor.class, "BL");
        BR = hardwareMap.get(DcMotor.class, "BR");

        Arm = hardwareMap.dcMotor.get("Arm");

        distanceSensor = hardwareMap.get(DistanceSensor.class,"distanceSensor");

        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        //Set Up Motor Direction
        FR.setDirection(DcMotor.Direction.FORWARD);
        FL.setDirection(DcMotor.Direction.REVERSE);
        BR.setDirection(DcMotor.Direction.FORWARD);
        BL.setDirection(DcMotor.Direction.REVERSE);


            telemetry.addData("STATUS", "Initialized");
            telemetry.update();

    }

    @Override
    public void loop() {
        double value = distanceSensor.getDistance(DistanceUnit.INCH);
        telemetry.addData("Distance",value);
        telemetry.update();

        drive = -gamepad1.left_stick_y *0.75;
        strafe = gamepad1.left_stick_x *0.75;
        rotate = -gamepad1.right_stick_x *0.75;

        FLPower = (drive + strafe + rotate);
        FRPower = (drive - strafe - rotate);
        BLPower = (drive - strafe + rotate);
        BRPower = (drive + strafe - rotate);

        FL.setPower(-FLPower);
        FR.setPower(-FRPower);
        BL.setPower(-BLPower);
        BR.setPower(-BRPower);


        if(gamepad1.dpad_up) {
            Arm.setPower(ArmPower);

        }
        else {
            Arm.setPower(0);
        }
        if (gamepad1.dpad_down) {
            Arm.setPower(-ArmPower);
        }
        else {
            Arm.setPower(0);
        }
//        if (value < 3) {
//            Arm.setPower(0.3);
//        }
//        else {
//            Arm.setPower(0);
//        }
    }
}




