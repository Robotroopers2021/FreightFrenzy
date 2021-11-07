package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServoImpl;
import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.Arrays;

@TeleOp
public class ControllerTest extends OpMode {

    DcMotor FL, FR, BL, BR;
    CRServoImpl LH;

    float FLPower;
    float FRPower;
    float BLPower;
    float BRPower;


    @Override
    public void init() {
        FL = hardwareMap.get(DcMotor.class, "FL");
        FR = hardwareMap.get(DcMotor.class, "FR");
        BL = hardwareMap.get(DcMotor.class, "BL");
        BR = hardwareMap.get(DcMotor.class, "BR");

        LH = (CRServoImpl) hardwareMap.get(CRServoImpl.class, "LH");

        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        FR.setDirection(DcMotor.Direction.REVERSE);
        FL.setDirection(DcMotor.Direction.FORWARD);
        BR.setDirection(DcMotor.Direction.FORWARD);
        BL.setDirection(DcMotor.Direction.REVERSE);


    }

    @Override
    public void loop() {

        FLPower = (gamepad1.left_stick_y + -gamepad1.right_stick_x);
        FRPower = (gamepad1.left_stick_y + gamepad1.right_stick_x);
        BLPower = (gamepad1.left_stick_y + -gamepad1.right_stick_x);
        BRPower = (gamepad1.left_stick_y + gamepad1.right_stick_x);

        FL.setPower(FLPower);
        FR.setPower(FRPower);
        BL.setPower(BLPower);
        BR.setPower(BRPower);

        LH.setPower(gamepad1.right_stick_y);
    }
}





















//        for (DcMotor dcMotor : Arrays.asList(FL, FR, BL, BR)) {
//            dcMotor.setPower(gamepad1.left_trigger);
//        }
//
//            FRPower = (gamepad1.right_stick_x);
//            BRPower = (-gamepad1.right_stick_x);
//
//            FR.setPower(FRPower);
//            BR.setPower(BRPower);






