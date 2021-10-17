package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

public class RobotroopersTest extends OpMode {

    DcMotor FL;
    @Override
    public void init() {
        FL = hardwareMap.get(DcMotor.class,"Frontleft");

        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

    @Override
    public void loop(){

    }
}
