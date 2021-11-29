package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.ControllerTest.ki;
import static org.firstinspires.ftc.teamcode.ControllerTest.kp;
import static org.firstinspires.ftc.teamcode.ControllerTest.kd;
import static org.firstinspires.ftc.teamcode.ControllerTest.max;
import static org.firstinspires.ftc.teamcode.ControllerTest.min;
import static org.firstinspires.ftc.teamcode.ControllerTest.targetPosition;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@Autonomous(name="Drive Avoid PID")
public class DriveAvoidPid extends LinearOpMode {

    DcMotor FR = null;
    DcMotor FL = null;
    DcMotor BR = null;
    DcMotor BL = null;
    DcMotor Arm = null;

    public ElapsedTime runtime = new ElapsedTime();
    public void drive() {

        double FLPower;
        double FRPower;
        double BLPower;
        double BRPower;

        double drive = 0;
        double strafe = 0;
        double rotate = 0;

        FLPower = (drive + strafe + rotate);
        FRPower = (drive - strafe - rotate);
        BLPower = ((drive - strafe + rotate));
        BRPower = ((drive + strafe - rotate));
    }
        PIDFController Controller = new PIDFController(new PIDCoefficients(kp, ki, kd));

    @Override
    public void runOpMode() throws InterruptedException {



        FR = hardwareMap.dcMotor.get("FR");
        FL = hardwareMap.dcMotor.get("FL");
        BR = hardwareMap.dcMotor.get("BR");
        BL = hardwareMap.dcMotor.get("BL");
        Arm = hardwareMap.dcMotor.get("Arm");

        FR.setDirection(DcMotor.Direction.FORWARD);
        FL.setDirection(DcMotor.Direction.REVERSE);
        BR.setDirection(DcMotor.Direction.FORWARD);
        BL.setDirection(DcMotor.Direction.REVERSE);

        Arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        telemetry.addData("Status", "Ready to run");
        telemetry.update();


        waitForStart();


        FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        FL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        TelemetryPacket packet = new TelemetryPacket();

        double output = Controller.update(Arm.getCurrentPosition());

        packet.put("target position", targetPosition);
        packet.put("Current Position",Arm.getCurrentPosition());
        FtcDashboard.getInstance().sendTelemetryPacket(packet);
        Arm.setPower(Range.clip(output,min,max));



        Trajectory doThings = new TrajectoryBuilder(new Pose2d(0, 0, 0), )
                .forward(24)
                .build();


        }

    }

