package org.firstinspires.ftc.teamcode;


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


     PIDFController armController = new PIDFController(new PIDCoefficients(kp,ki,kd));

    //Identifying Motors and Servos
    DcMotor FL, FR, BL, BR, Intake, duckL;
    Servo outtake;
    DcMotor arm;
    DistanceSensor lock;

    double drive;
    double strafe;
    double rotate;

    double FLPower;
    double FRPower;
    double BLPower;
    double BRPower;

    double intakePower;

    double duckLPower =0.75;

    double armPower;

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
    public double currentPosition = 0;

    private void armControl() {
        currentPosition = arm.getCurrentPosition()-114;
        feedForward = getFeedForward(Math.toRadians((targetAngle)));
        pidOutput = armController.update(currentPosition);
        output = feedForward + pidOutput;
        if (gamepad1.dpad_up) {
            moveArmToDegree(depositAngle);
            outtake.setPosition(0.75);
        }
        if (gamepad1.dpad_down) {
            moveArmToDegree(restAngle);
            outtake.setPosition(0.9);
        }
        arm.setPower(output);
    }


    private void driveControl() {
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
    }

    private void intakeControl() {
        intakePower = gamepad1.left_trigger;
        Intake.setPower(-intakePower * 0.85);

    }

    private void outtakeControl() {
        double value = lock.getDistance(DistanceUnit.INCH);
        if (gamepad1.a) {
            outtake.setPosition(0.9);
        }
        if (gamepad1.b) {
            outtake.setPosition(0.6);
        }
        if (value <2) {
            outtake.setPosition(0.74);
        }
    }

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

        lock = hardwareMap.get(DistanceSensor.class,"Lock");

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

        TelemetryPacket packet = new TelemetryPacket();
        packet.put("target angle", targetAngle);
        packet.put("output", output);
        packet.put("Current Position",currentPosition);
        packet.put("degrees", currentPosition * degreesPerTick);
        packet.put("feedforward", getFeedForward(Math.toRadians(targetAngle)));
        packet.put("target ticks", targetTicks);
        FtcDashboard.getInstance().sendTelemetryPacket(packet);

        outtake.setPosition(0.9);
        armController.reset();
        targetAngle = restAngle;
        targetTicks = restAngle * ticksPerDegree;
        armController.setTargetPosition(targetTicks);


        telemetry.addData("STATUS", "Initialized");
        telemetry.update();

    }

    @Override
    public void loop() {
        double value = lock.getDistance(DistanceUnit.INCH);
        driveControl();
        armControl();
        intakeControl();
        outtakeControl();
        duckControl();




        //Adding Telemetry
        //telemetry.addData("Servo Position", Outtake.getPosition());
        telemetry.addData("Motor Power", FL.getPower());
        telemetry.addData("Motor Power", FR.getPower());
        telemetry.addData("Motor Power", BL.getPower());
        telemetry.addData("Motor Power", BR.getPower());
        telemetry.addData("Motor Power", arm.getPower());
        telemetry.addData("FL Power", FLPower);
        telemetry.addData("BR Power", BRPower);
        telemetry.addData("BL Power", BLPower);
        telemetry.addData("FR Power", FRPower);
        telemetry.addData("Arm Power", armPower);
        telemetry.addData("Status", "Running");
        telemetry.addData("Status", arm.getPower());
        telemetry.addData("Arm", currentPosition);
        telemetry.addData("Outtake", outtake.getPosition());
        telemetry.addData("Distance",value);
        telemetry.update();

    }
}























//        for (DcMotor dcMotor : Arrays.asList(FL, FR, BL, BR)) {
//            dcMotor.setPower(gamepad1.left_trigger);
//        }







