package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.Arrays;

@TeleOp
public class ControllerTest extends OpMode {

    //Identifying Motors and Servos
    DcMotor FL, FR, BL, BR;
    Servo LH;
    //Create Motor Power
    float FLPower;
    float FRPower;
    float BLPower;
    float BRPower;


    @Override
    public void init() {
        //Connect Motor
        FL = hardwareMap.get(DcMotor.class, "FL");
        FR = hardwareMap.get(DcMotor.class, "FR");
        BL = hardwareMap.get(DcMotor.class, "BL");
        BR = hardwareMap.get(DcMotor.class, "BR");
        //Connect Servo
        LH = (Servo) hardwareMap.get(Servo.class, "LH");
        //Set ZERO POWER BEHAVIOR
        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        //Set Up Motor Direction
        FR.setDirection(DcMotor.Direction.REVERSE);
        FL.setDirection(DcMotor.Direction.FORWARD);
        BR.setDirection(DcMotor.Direction.FORWARD);
        BL.setDirection(DcMotor.Direction.REVERSE);


    }

    @Override
    public void loop() {
        //Link Motors to Joysticks
        FLPower = (gamepad1.left_stick_y + -gamepad1.right_stick_x);
        FRPower = (gamepad1.left_stick_y + gamepad1.right_stick_x);
        BLPower = (gamepad1.left_stick_y + -gamepad1.right_stick_x);
        BRPower = (gamepad1.left_stick_y + gamepad1.right_stick_x);
        //Set Motor Power
        FL.setPower(FLPower);
        FR.setPower(FRPower);
        BL.setPower(BLPower);
        BR.setPower(BRPower);
        //Link Servo to Controller Buttons
        if (gamepad1.y) {
            // move to 0 degrees.
            LH.setPosition(0);
        } else if (gamepad1.x) {
            // move to 90 degrees.
            LH.setPosition(0.5);
        } else if (gamepad1.b) {
            //move to 90 degrees opposite direction
            LH.setPosition(-0.5);
        }else if (gamepad1.a) {
            // move to 180 degrees.
            LH.setPosition(1);
        }
        //Adding Telemetry
        telemetry.addData("Servo Position", LH.getPosition());
        telemetry.addData("Motor Power", FL.getPower());
        telemetry.addData("Motor Power", FR.getPower());
        telemetry.addData("Motor Power", BL.getPower());
        telemetry.addData("Motor Power", BR.getPower());
        telemetry.addData("Status", "Running");
        telemetry.update();

    }
}























//        for (DcMotor dcMotor : Arrays.asList(FL, FR, BL, BR)) {
//            dcMotor.setPower(gamepad1.left_trigger);
//        }







