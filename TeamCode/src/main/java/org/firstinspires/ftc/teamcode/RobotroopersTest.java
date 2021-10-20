package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.Arrays;

@TeleOp
public class RobotroopersTest extends OpMode {

    DcMotor FL, FR, BL, BR;

    float FLPower;
    float FRPower;
    float BLPower;
    float BRPower;

    @Override
    public void init() {
        FL = hardwareMap.get(DcMotor.class, "Frontleft");
        FR = hardwareMap.get(DcMotor.class, "Frontright");
        BL = hardwareMap.get(DcMotor.class, "Backleft");
        BR = hardwareMap.get(DcMotor.class, "Backright");

        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    @Override
    public void loop() {
        boolean left_stick_y = true;
        FLPower = (gamepad1.left_stick_y);
        FRPower = (gamepad1.left_stick_y);
        BLPower = (gamepad1.left_stick_y);
        BRPower = (gamepad1.left_stick_y);
        {
            FL.setPower(FLPower);
            FR.setPower(FRPower);
            BL.setPower(BLPower);
            BR.setPower(BRPower);
        }


    }
        }

