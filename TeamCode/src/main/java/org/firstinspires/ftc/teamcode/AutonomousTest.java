package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous (name= "AutonomousTest")
public class AutonomousTest extends LinearOpMode {

    DcMotor FR = null;
    DcMotor FL = null;
    DcMotor BR = null;
    DcMotor BL = null;
    DcMotor Arm = null;
//    DcMotor Intake = null;

    Servo LH = null;

    //Convert from counts per revolution to counts per inch
    static final double HD_COUNTS_PER_REV = 28;
    static final double DRIVE_GEAR_REDUCTION = 1.0;
    static final double WHEEL_CIRCUMFERENCE_IN = 4 * Math.PI;
    static final double DRIVE_COUNTS_PER_IN = (HD_COUNTS_PER_REV * DRIVE_GEAR_REDUCTION) / WHEEL_CIRCUMFERENCE_IN;

    //Create elapsed time variable and an instance of elapsed time
    public ElapsedTime runtime = new ElapsedTime();

    //Drive function with 3 parameters
    public void drive(double drive, double strafe, double rotate) {
        int FRTarget;
        int BRTarget;
        int FLTarget;
        int BLTarget;

        double FLPower;
        double FRPower;
        double BLPower;
        double BRPower;


        FLPower = (drive + strafe + rotate);
        FRPower = (drive - strafe - rotate);
        BLPower = ((drive - strafe + rotate));
        BRPower = ((drive + strafe - rotate));
        if (opModeIsActive()) {

            FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            //Create Target Positions
            FRTarget = FR.getCurrentPosition() + (int) (FRPower * DRIVE_COUNTS_PER_IN);
            BRTarget = BR.getCurrentPosition() + (int) (BRPower * DRIVE_COUNTS_PER_IN);
            FLTarget = FL.getCurrentPosition() + (int) (FLPower * DRIVE_COUNTS_PER_IN);
            BLTarget = BL.getCurrentPosition() + (int) (BLPower * DRIVE_COUNTS_PER_IN);

            //Set Target Position
            FR.setTargetPosition(FRTarget);
            BR.setTargetPosition(BRTarget);
            FL.setTargetPosition(FLTarget);
            BL.setTargetPosition(BLTarget);

            //Switch to run to position mode
            FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BL.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            //run to position at the designated power
            FL.setPower(FLPower);
            FR.setPower(FRPower);
            BL.setPower(BLPower);
            BR.setPower(BRPower);

            //wait until both motors are no longer busy running to position
            while (opModeIsActive() && (FL.isBusy() || BL.isBusy() || FR.isBusy() || BR.isBusy())) {
            }

            //set motor power back to 0
            FR.setPower(0);
            FL.setPower(0);
            BR.setPower(0);
            BL.setPower(0);

        }
    }

    @Override
    public void runOpMode() throws InterruptedException {

        FR = hardwareMap.dcMotor.get("FR");
        FL = hardwareMap.dcMotor.get("FL");
        BR = hardwareMap.dcMotor.get("BR");
        BL = hardwareMap.dcMotor.get("BL");
        Arm = hardwareMap.dcMotor.get("Arm");
//        Intake = hardwareMap.dcMotor.get("Intake");

        LH = (Servo) hardwareMap.get(Servo.class, "LH");

        FR.setDirection(DcMotor.Direction.FORWARD);
        FL.setDirection(DcMotor.Direction.REVERSE);
        BR.setDirection(DcMotor.Direction.FORWARD);
        BL.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addData("FL Target Position", FL.getTargetPosition());
        telemetry.addData("FR Target Position", FR.getTargetPosition());
        telemetry.addData("BL Target Position", BL.getTargetPosition());
        telemetry.addData("BR Target Position", BR.getTargetPosition());
        telemetry.addData("Arm Target Position",Arm.getTargetPosition());
        telemetry.update();

        waitForStart();
        if (opModeIsActive()) {

            drive(1600,0,0);

            drive(0,0,-900);

            Arm.setTargetPosition(60);
            Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Arm.setPower(0.3);

            drive(0,-1600,0);


            //segment 1
//            drive(0.7, 30, 15);
//
//            runtime.reset();
//
//            //segment 2 - lift arm, drive to shipping hub, outtake freight
////            while (opModeIsActive() && runtime.seconds() <= 7) {
////
//            //lift arm and hold
//                Arm.setTargetPosition(120);
//                Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                Arm.setPower(0.3);
//
//            //drive forward for 1 second
//            while (runtime.seconds() > 2 && runtime.seconds() <= 3) {
//                drive(1,200, 200);
//            }
//
//            //run intake
////            while (runtime.seconds() > 4 && runtime.seconds() <= 7) {
////            Intake.setPower(-0.6);
////            }
//
//            //turn off arm and intake
//            Arm.setPower(0);
////            Intake.setPower(0);
//
//            //segment 3 - reverse to get better angle
//            drive(0.7,-15,-30);
//
//            //segment 4 - drive into warehouse
//            drive(1,1000,1900);
        }
    }
}

