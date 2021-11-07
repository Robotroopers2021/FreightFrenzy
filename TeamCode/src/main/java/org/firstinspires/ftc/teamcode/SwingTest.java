package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import java.util.Arrays;

@TeleOp
public class SwingTest extends OpMode {

    DcMotor  FR;


    float FRPower;


    @Override
    public void init() {

        FR = hardwareMap.get(DcMotor.class, "FR");



        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);


        FR.setDirection(DcMotor.Direction.REVERSE);


    }

    @Override
    public void loop() {


        FRPower  = (gamepad1.left_stick_y + gamepad1.right_stick_x);



        FR.setPower(FRPower);




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


}




