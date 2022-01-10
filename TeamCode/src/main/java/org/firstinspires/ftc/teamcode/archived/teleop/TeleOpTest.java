package org.firstinspires.ftc.teamcode.archived.teleop;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Config
@TeleOp (name = "TeleOpTest")
public class TeleOpTest extends OpMode {

    public static double kp = 0.015;
    public static double ki = 0;
    public static double kd = 0;
    public static double targetPosition = 410;
    public static double min = -0.5;
    public static double max = 0.5;
//    public  static double kcos;
//    public static double kv;
//    public static double targetVelocity =415;
//    public static double targetAngle;
//
//    double reference = targetAngle;

    PIDFController Controller = new PIDFController(new PIDCoefficients(kp,ki,kd));

    //Identifying Motors and Servos
    DcMotor FL, FR, BL, BR, Intake,DuckL;
    Servo Outtake;
    DcMotor Arm;
    DistanceSensor Lock;

    double drive;
    double strafe;
    double rotate;

    double FLPower;
    double FRPower;
    double BLPower;
    double BRPower;

    double IntakePower;

    double DuckLPower =0.75;

    double ArmPower;

    @Override
    public void init() {
        //Connect Motor
        FL = hardwareMap.get(DcMotor.class, "FL");
        FR = hardwareMap.get(DcMotor.class, "FR");
        BL = hardwareMap.get(DcMotor.class, "BL");
        BR = hardwareMap.get(DcMotor.class, "BR");


        Arm = hardwareMap.dcMotor.get("Arm");

        DuckL = hardwareMap.get(DcMotor.class, "DuckL");

        Intake = hardwareMap.dcMotor.get("Intake");

        Lock = hardwareMap.get(DistanceSensor.class,"Lock");

        //Connect Servo
        Outtake = (Servo) hardwareMap.get(Servo.class, "Outtake");
        //Set ZERO POWER BEHAVIOR
        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //Set Up Motor Direction
        FR.setDirection(DcMotor.Direction.FORWARD);
        FL.setDirection(DcMotor.Direction.REVERSE);
        BR.setDirection(DcMotor.Direction.FORWARD);
        BL.setDirection(DcMotor.Direction.REVERSE);

        Arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Outtake.setPosition(0.9);
        telemetry.addData("STATUS", "Initialized");
        telemetry.update();

    }

    //    double getoutput() {
//        return Math.cos(reference)*kcos;
//
//
//    }
//    double getvelocity() {
//        return kv * targetVelocity;
//    }
    @Override
    public void loop() {
        double output = Controller.update(Arm.getCurrentPosition());

        Arm.setPower(Range.clip(output,min,max));




        if(gamepad1.dpad_up) {
            Controller.setTargetPosition(targetPosition);
        }
        if (gamepad1.dpad_down) {
            Controller.setTargetPosition(65);
            Outtake.setPosition(0.9);
        }

        double value = Lock.getDistance(DistanceUnit.INCH);

        TelemetryPacket packet = new TelemetryPacket();

        drive = -gamepad1.left_stick_y;
        strafe = gamepad1.left_stick_x;
        rotate = gamepad1.right_stick_x;

        FLPower = (drive + strafe + rotate);
        FRPower = (drive - strafe - rotate);
        BLPower = (drive - strafe + rotate);
        BRPower = (drive + strafe - rotate);

        FL.setPower(FLPower);
        FR.setPower(FRPower);
        BL.setPower(BLPower);
        BR.setPower(BRPower);

        IntakePower = gamepad1.left_trigger;
        Intake.setPower(-IntakePower * 0.75);

        if (gamepad1.left_bumper) {
            DuckL.setPower(DuckLPower);
        } else {
            DuckL.setPower(0);
        }

        if (gamepad1.right_bumper) {
            DuckL.setPower(-DuckLPower);
        } else {
            DuckL.setPower(0);
        }

        if (gamepad1.a) {
            Outtake.setPosition(0.6);
        }
        if (gamepad1.b) {
            Outtake.setPosition(0.9);
        }

        if (value <2) {
            Outtake.setPosition(0.75);
        }



        //Adding Telemetry
        //telemetry.addData("Servo Position", Outtake.getPosition());
        telemetry.addData("Motor Power", FL.getPower());
        telemetry.addData("Motor Power", FR.getPower());
        telemetry.addData("Motor Power", BL.getPower());
        telemetry.addData("Motor Power", BR.getPower());
        telemetry.addData("Motor Power", Arm.getPower());
        telemetry.addData("FL Power", FLPower);
        telemetry.addData("BR Power", BRPower);
        telemetry.addData("BL Power", BLPower);
        telemetry.addData("FR Power", FRPower);
        telemetry.addData("Arm Power", ArmPower);
        telemetry.addData("Status", "Running");
        telemetry.addData("Status", Arm.getPower());
        telemetry.addData("Arm", Arm.getCurrentPosition());
        telemetry.addData("Distance",value);
        telemetry.addData("Outtake",Outtake.getPosition());
        telemetry.update();

    }
}