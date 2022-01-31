package org.firstinspires.ftc.teamcode.archived.teleop

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.hardware.rev.Rev2mDistanceSensor
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.teamcode.util.math.MathUtil

@Config
@TeleOp
class LeviTeleOp : OpMode() {


    lateinit var fl: DcMotor
    lateinit var fr: DcMotor
    lateinit var bl: DcMotor
    lateinit var br: DcMotor


    private lateinit var intakeMotor: DcMotor

    private lateinit var duck: DcMotor

    lateinit var outtakeServo: Servo

    private val arm = Arm()

    private lateinit var distanceSensor : Rev2mDistanceSensor

    var drive = 0.0
    private var strafe = 0.0
    var rotate = 0.0

    private lateinit var jugaad : Jugaad


    private fun driveControl() {
        drive = MathUtil.cubicScaling(0.85, -gamepad1.left_stick_y.toDouble()) * 0.75
        strafe = MathUtil.cubicScaling(0.85, gamepad1.left_stick_x.toDouble()) * 0.75
        rotate = MathUtil.cubicScaling(0.85, gamepad1.right_stick_x.toDouble()) * 0.6
        fl.power = drive + strafe + rotate
        fr.power = drive - strafe - rotate
        bl.power = drive - strafe + rotate
        br.power = drive + strafe - rotate
    }

    private fun armControl() {
        when {
            gamepad1.left_bumper -> {
                arm.moveArmToTopPos()
            }
            gamepad1.right_bumper -> {
                arm.moveArmToBottomPos()
                outtakeServo.position = 0.92
            }
            gamepad1.b -> {
                arm.moveArmToSharedPosBalanced()
            }
        }
    }

    private fun intakeControl() {

        jugaad.runIntakeSequence(gamepad1.a)

        when {
            gamepad1.right_trigger > 0.25 -> {
                jugaad.reverseIntake()
            }
            gamepad1.left_trigger > 0.25 -> {
                jugaad.intakeFreight()
            }
            else -> {
                jugaad.stopIntake()
            }
        }
    }

    private fun outtakeControl() {
        if (gamepad2.a) {
            jugaad.moveOuttakeToOpen()
        }
        if (gamepad2.b) {
            jugaad.moveOuttakeToDeposit()
        }
        if (gamepad2.x) {
            jugaad.moveOuttakeToLock()
        }
    }

    private fun duckControl() {
        when {
            gamepad1.dpad_left -> {
                duck.power = -0.75
            }
            gamepad1.dpad_right -> {
                duck.power = 0.75
            }
            else -> {
                duck.power = 0.0
            }
        }
    }

    private fun dSensorControl () {
        val dsDistance = distanceSensor.getDistance(DistanceUnit.INCH)

        telemetry.addData("Distance",dsDistance)
        telemetry.update()

    }


    override fun init() {
        //Connect Motor
        fl = hardwareMap.get(DcMotor::class.java, "FL")
        fr = hardwareMap.get(DcMotor::class.java, "FR")
        bl = hardwareMap.get(DcMotor::class.java, "BL")
        br = hardwareMap.get(DcMotor::class.java, "BR")
        duck = hardwareMap.get(DcMotor::class.java, "DuckL")
        intakeMotor = hardwareMap.dcMotor["Intake"]
        intakeMotor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE

        distanceSensor = hardwareMap.get(Rev2mDistanceSensor::class.java, "distanceSensor") as Rev2mDistanceSensor

        fl.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        fr.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        bl.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        br.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE

        fl.direction = DcMotorSimple.Direction.REVERSE
        bl.direction = DcMotorSimple.Direction.REVERSE

        outtakeServo = hardwareMap.get(Servo::class.java, "Outtake") as Servo

        arm.init(hardwareMap)

        jugaad = Jugaad(
            intakeMotor, outtakeServo, distanceSensor, arm,
        )

        jugaad.moveOuttakeToOpen()
    }

    override fun loop() {
        driveControl()
        intakeControl()
        outtakeControl()
        duckControl()
        dSensorControl()
        armControl()
        arm.update()
        arm.updateTelemetry()

    }

}