package org.firstinspires.ftc.teamcode.archived.auto

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.archived.teleop.Arm
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive
import org.firstinspires.ftc.teamcode.stateMachine.StateMachineBuilder
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence
import org.firstinspires.ftc.teamcode.vision.AutoWebcamTest
import org.firstinspires.ftc.teamcode.vision.Pipeline
import org.firstinspires.ftc.teamcode.vision.WebcamTest

@Autonomous(preselectTeleOp = "AkazaBlueOp")
class AutoPipelineTest : OpMode()  {


    private val webcam = AutoWebcamTest()

    lateinit var drive: SampleMecanumDrive




    override fun init() {
        drive = SampleMecanumDrive(hardwareMap)
        webcam.init(hardwareMap)
    }

    override fun init_loop() {
        super.init_loop()
        webcam.update()
        telemetry.addData("val", webcam.pipeline.total)
        telemetry.addData("armpos", webcam.armpos)
        telemetry.update()
    }

    override fun start() {
        super.start()
        webcam.reset()
    }

    override fun loop() {
        drive.update()

    }
}