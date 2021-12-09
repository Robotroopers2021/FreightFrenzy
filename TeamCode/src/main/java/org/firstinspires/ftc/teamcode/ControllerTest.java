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
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Config
@TeleOp (name = "ControllerTest")
public class ControllerTest extends OpMode {

    public static double kp = 0.015;
    public static double ki = 0;
    public static double kd = 0.00065;
    public static double targetPosition = 296;
    public static double min = -0.5;
    public static double max = 0.5;
    public  static double kcos =0.5;
    public static double kv;


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
        Arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        Outtake.setPosition(0.9);

        telemetry.addData("STATUS", "Initialized");
        telemetry.update();

    }

    double getoutput(double targetAngle,double targetVelocity) {
        return Math.cos(targetAngle)*kcos + kv * targetVelocity;
    }
    double degreesPerTick = 90/184.0;
    double output;
    @Override
    public void loop() {

        double currentPosition = Arm.getCurrentPosition()-114;

        output =getoutput(Math.toRadians(targetPosition* degreesPerTick),10)
                + Controller.update(currentPosition);




        Arm.setPower(output);

        if(gamepad1.dpad_up) {
            Controller.reset();
            Controller.setTargetPosition(targetPosition);
        }
        if (gamepad1.dpad_down) {
            Controller.reset();
            Controller.setTargetPosition(30);
            Outtake.setPosition(0);
        }

        double value = Lock.getDistance(DistanceUnit.INCH);
        TelemetryPacket packet = new TelemetryPacket();
        packet.put("target position", targetPosition);
        packet.put("output", output);
        packet.put("Current Position",currentPosition);
        packet.put("degrees", currentPosition * degreesPerTick);
        packet.put("feedforward", getoutput(Math.toRadians(targetPosition * degreesPerTick), 0));
        FtcDashboard.getInstance().sendTelemetryPacket(packet);

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
            Outtake.setPosition(0.9);
        }

        if(gamepad1.b) {
            Outtake.setPosition(0.6);
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
        telemetry.addData("Arm", currentPosition);
        telemetry.addData("Outtake",Outtake.getPosition());
        telemetry.addData("Distance",value);
        telemetry.update();

    }
}























//        for (DcMotor dcMotor : Arrays.asList(FL, FR, BL, BR)) {
//            dcMotor.setPower(gamepad1.left_trigger);
//        }







