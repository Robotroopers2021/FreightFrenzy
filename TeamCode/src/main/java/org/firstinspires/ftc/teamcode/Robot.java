package org.firstinspires.ftc.teamcode;

import static com.qualcomm.hardware.lynx.LynxModule.*;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Drive;

import java.util.logging.Logger;

public abstract class Robot {

    HardwareMap hardwareMap;
    OpMode opMode;
    Telemetry telemetry;

    public Drive driveTrain;

    public Robot(OpMode op) {
        opMode = op;
        hardwareMap = opMode.hardwareMap;
        telemetry = opMode.telemetry;

        for (LynxModule module : hardwareMap.getAll(LynxModule.class))
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
    }

    public void sleep(long millis) {
        long startTime = System.currentTimeMillis();
        while (System.currentTimeMillis() < startTime + millis && opModeIsActive()) ;
    }

    public boolean opModeIsActive() {
        try {
            return ((LinearOpMode) opMode).opModeIsActive();
        } catch (ClassCastException e) {
            return true;
        }

    }
}
