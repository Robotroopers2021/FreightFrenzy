package org.firstinspires.ftc.teamcode.archived.teleop;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Config
@TeleOp (name = "ControllerTest")
public class ControllerTest extends OpMode {

    public static double kp = 0.015;
    public static double ki = 0;
    public static double kd = 0.00075;
    public static double targetAngle = 296;
    public  static double kcos =0.275;
    public static double kv;
    public static double depositAngle =140;
    public static double restAngle = -55;

    public BNO055IMU imu;


     PIDFController armController = new PIDFController(new PIDCoefficients(kp,ki,kd));

    //Identifying Motors and Servos
    DcMotor FL, FR, BL, BR, Intake, duckL;
    Servo outtake;
    DcMotor arm;
    //DistanceSensor lock;

    double drive;
    double strafe;
    double rotate;

    double FLPower;
    double FRPower;
    double BLPower;
    double BRPower;

    double intakePower;

    double duckLPower =0.75;

    public double currentPosition = 0;

    double degreesPerTick =90/184.0;
    double ticksPerDegree = 184.0/90;
    double targetTicks =0.0;
    double output =0.0;
    double pidOutput =0.0;
    double feedForward = 0.0;

    private void moveArmToDegree(double degrees)  {
        targetAngle =degrees;
        targetTicks = targetAngle * ticksPerDegree;
        armController.reset();
        armController.setTargetPosition(targetTicks);
    }

    private double getFeedForward(double targetAngle) {
        return Math.cos(targetAngle) * kcos;
    }

    private void armControl() {
        if (gamepad1.dpad_up) {
            moveArmToDegree(depositAngle);
            outtake.setPosition(0.8);
        }
        if (gamepad1.dpad_down) {
            moveArmToDegree(restAngle);
            outtake.setPosition(0.9);
        }

        feedForward = getFeedForward(Math.toRadians(targetAngle));
        pidOutput = armController.update(currentPosition);
        output = feedForward + pidOutput;
        arm.setPower(output);
    }

    private void currentPos() {
        currentPosition = arm.getCurrentPosition()-114;
    }


    private void driveControl() {

        double driveTurn = -gamepad1.left_stick_x;

        double gamepadXCoordinate = gamepad1.right_stick_x;
        double gamepadYCoordinate = -gamepad1.right_stick_y;
        double gamepadHypot = Range.clip(Math.hypot(gamepadXCoordinate, gamepadYCoordinate), 0, 1);

        double gamepadDegree = Math.atan2(gamepadYCoordinate, gamepadXCoordinate);

        //double robotDegree = getAngle();

        //double movementDegree = gamepadDegree - robotDegree;

//        double gamepadXControl = Math.cos(Math.toRadians(movementDegree)) * gamepadHypot;
//
//        double gamepadYControl = Math.sin(Math.toRadians(movementDegree)) * gamepadHypot;

//        FR.setPower(gamepadYControl * Math.abs(gamepadYControl) - gamepadXControl * Math.abs(gamepadXControl) + driveTurn);
//        BR.setPower(gamepadYControl * Math.abs(gamepadYControl) + gamepadXControl * Math.abs(gamepadXControl) + driveTurn);
//        FL.setPower(gamepadYControl * Math.abs(gamepadYControl) + gamepadXControl * Math.abs(gamepadXControl) - driveTurn);
//        BL.setPower(gamepadYControl * Math.abs(gamepadYControl) - gamepadXControl * Math.abs(gamepadXControl) - driveTurn);
        

        drive = -gamepad1.left_stick_y *0.75;
        strafe = gamepad1.left_stick_x *0.75;
        rotate = gamepad1.right_stick_x *0.6;

        FLPower = (drive + strafe + rotate);
        FRPower = (drive - strafe - rotate);
        BLPower = (drive - strafe + rotate);
        BRPower = (drive + strafe - rotate);

        FL.setPower(FLPower);
        FR.setPower(FRPower);
        BL.setPower(BLPower);
        BR.setPower(BRPower);
    }

    private void intakeControl() {
        intakePower = gamepad1.left_trigger + -gamepad1.right_trigger;
        Intake.setPower(-intakePower * 0.85);


    }

    private void outtakeControl() {
        //double value = lock.getDistance(DistanceUnit.INCH);
        if (gamepad1.a) {
            outtake.setPosition(0.9);
        }
        if (gamepad1.b) {
            outtake.setPosition(0.6);
        }
//        if (value <2) {
//            outtake.setPosition(0.8);
        }
    //}

    private void duckControl() {
        if(gamepad1.left_bumper) {
            duckL.setPower(duckLPower);
        }
        else if (gamepad1.right_bumper) {
            duckL.setPower(-duckLPower);
        } else {
            duckL.setPower(0);
        }
    }


    @Override
    public void init() {
        //Connect Motor
        FL = hardwareMap.get(DcMotor.class, "FL");
        FR = hardwareMap.get(DcMotor.class, "FR");
        BL = hardwareMap.get(DcMotor.class, "BL");
        BR = hardwareMap.get(DcMotor.class, "BR");


        arm = hardwareMap.dcMotor.get("Arm");

        duckL = hardwareMap.get(DcMotor.class, "DuckL");

        Intake = hardwareMap.dcMotor.get("Intake");

        //lock = hardwareMap.get(DistanceSensor.class,"Lock");


        //Connect Servo
        outtake = (Servo) hardwareMap.get(Servo.class, "Outtake");
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

        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);


        outtake.setPosition(0.9);
        targetAngle = restAngle;
        targetTicks = restAngle * ticksPerDegree;


        telemetry.addData("STATUS", "Initialized");
        telemetry.update();

    }

    @Override
    public void loop() {
        //double value = lock.getDistance(DistanceUnit.INCH);
        driveControl();
        currentPos();
        intakeControl();
        outtakeControl();
        duckControl();

        telemetry.addData("Current Position",currentPosition);
        telemetry.update();

        TelemetryPacket packet = new TelemetryPacket();
        packet.put("target angle", targetAngle);
        packet.put("pid output", pidOutput);
        packet.put("output", pidOutput + feedForward);
        packet.put("degrees", currentPosition * degreesPerTick);
        packet.put("feedforward", feedForward);
        packet.put("target ticks", targetTicks);
        FtcDashboard.getInstance().sendTelemetryPacket(packet);


    }
}