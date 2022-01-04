package org.firstinspires.ftc.teamcode

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.control.PIDCoefficients
import com.acmerobotics.roadrunner.control.PIDFController
import com.qualcomm.hardware.bosch.BNO055IMU
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.util.Range
import org.firstinspires.ftc.robotcore.external.navigation.*
import kotlin.math.*
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator




@Config
@TeleOp
class AkazaTeleopForSonny : OpMode() {
    lateinit var fl: DcMotor
    lateinit var fr: DcMotor
    lateinit var bl: DcMotor
    lateinit var br: DcMotor

    lateinit var intakeMotor: DcMotor

    lateinit var duck: DcMotor

    lateinit var arm: DcMotor
    lateinit var outtakeServo: Servo

    lateinit var imu: BNO055IMU
    lateinit var angles: Orientation
    lateinit var gravity: Acceleration

    var drive = 0.0
    var strafe = 0.0
    var rotate = 0.0
    var duckPower = 0.75

    var driveTurn: Double = 0.0
    var gamepadXCoordinate: Double = 0.0
    var gamepadYCoordinate: Double = 0.0
    var gamepadHypot: Double = 0.0
    var gamepadDegree: Double = 0.0
    var robotDegree: Double = 0.0
    var movementDegree: Double = 0.0
    var gamepadXControl: Double = 0.0
    var gamepadYControl: Double = 0.0

    var armController = PIDFController(PIDCoefficients(kp, ki, kd))

    var degreesPerTick = 90 / 184.0
    var ticksPerDegree = 184.0 / 90
    var targetTicks = 0.0
    var output = 0.0
    var pidOutput = 0.0
    var feedForward = 0.0

    private fun moveArmToDegree(degrees: Double) {
        targetAngle = degrees
        targetTicks = targetAngle * ticksPerDegree
        armController.reset()
        armController.targetPosition = targetTicks
    }

    private fun getFeedForward(targetAngle: Double): Double {
        return cos(targetAngle) * kcos
    }

    private fun armControl() {
        when {
            gamepad1.left_bumper -> {
                moveArmToDegree(depositAngle)
            }
            gamepad1.right_bumper -> {
                moveArmToDegree(restAngle)
                outtakeServo.position = 0.92
            }
            gamepad1.a -> {
                moveArmToDegree(sharedAngle)
            }
            gamepad1.x -> {
                moveArmToDegree(sharedAngleAlliance)
            }
            gamepad1.b -> {
                moveArmToDegree(sharedAngleEnemy)
            }
        }


        val currentPosition = (arm.currentPosition - 114).toDouble()

        feedForward = getFeedForward(Math.toRadians(targetAngle))
        pidOutput = armController.update(currentPosition)
        output = feedForward + pidOutput

        arm.power = output


        val packet = TelemetryPacket()
        packet.put("target angle", targetAngle)
        packet.put("output", output)
        packet.put("Current Position", currentPosition)
        packet.put("degrees", currentPosition * degreesPerTick)
        packet.put("feedforward", getFeedForward(Math.toRadians(targetAngle)))
        packet.put("target ticks", targetTicks)
        FtcDashboard.getInstance().sendTelemetryPacket(packet)
    }
    private fun getAngle(): Double {
        return imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle.toDouble()
    }

    private fun driveControl() {
        imu.startAccelerationIntegration(Position(), Velocity(), 1000)
        driveTurn = -gamepad1.right_stick_x.toDouble()
        gamepadXCoordinate = gamepad1.left_stick_x.toDouble()
        gamepadYCoordinate = -gamepad1.left_stick_y.toDouble()
        gamepadHypot = Range.clip(hypot(gamepadXCoordinate, gamepadYCoordinate), 0.0, 1.0)
        gamepadDegree = atan2(gamepadYCoordinate, gamepadXCoordinate)
        gravity = imu.gravity
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES)
        robotDegree = getAngle()
        movementDegree = gamepadDegree - robotDegree
        gamepadXControl = cos(Math.toRadians(movementDegree)) * gamepadHypot
        gamepadYControl = sin(Math.toRadians(movementDegree)) * gamepadHypot

        fr.power = gamepadYControl * abs(gamepadYControl) - gamepadXControl * abs(gamepadXControl) + driveTurn
        br.power = gamepadYControl * abs(gamepadYControl) + gamepadXControl * abs(gamepadXControl) + driveTurn
        fl.power = gamepadYControl * abs(gamepadYControl) + gamepadXControl * abs(gamepadXControl) - driveTurn
        bl.power = gamepadYControl * abs(gamepadYControl) - gamepadXControl * abs(gamepadXControl) - driveTurn
    }

    private fun intakeControl() {

        when {
            gamepad1.right_trigger > 0.5 -> {
                intakeMotor.power = 1.0
            }
            gamepad1.left_trigger > 0.5 -> {
                intakeMotor.power = -1.0
            }
            else -> {
                intakeMotor.power = 0.0
            }
        }
    }

    private fun outtakeControl() {
        if (gamepad2.a) {
            outtakeServo.position = 0.92
        }
        if (gamepad2.b) {
            outtakeServo.position = 0.6
        }
        if (gamepad2.x) {
            outtakeServo.position = 0.8
        }
    }

    private fun duckControl() {
        when {
            gamepad1.dpad_left -> {
                duck.power = duckPower
            }
            gamepad1.dpad_right -> {
                duck.power = -duckPower
            }
            else -> {
                duck.power = 0.0
            }
        }
    }

    override fun init() {
        //Connect Motor
        fl = hardwareMap.get(DcMotor::class.java, "FL")
        fr = hardwareMap.get(DcMotor::class.java, "FR")
        bl = hardwareMap.get(DcMotor::class.java, "BL")
        br = hardwareMap.get(DcMotor::class.java, "BR")
        arm = hardwareMap.dcMotor["Arm"]
        duck = hardwareMap.get(DcMotor::class.java, "DuckL")
        intakeMotor = hardwareMap.dcMotor["Intake"]
        intakeMotor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE

        outtakeServo = hardwareMap.get(Servo::class.java, "Outtake") as Servo

        fl.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        fr.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        bl.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        br.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE

        fl.direction = DcMotorSimple.Direction.REVERSE
        bl.direction = DcMotorSimple.Direction.REVERSE

        arm.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        arm.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        arm.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.FLOAT

        val parameters = BNO055IMU.Parameters()
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"
        parameters.loggingEnabled = true
        parameters.loggingTag = "IMU"
        parameters.accelerationIntegrationAlgorithm = JustLoggingAccelerationIntegrator()

        imu = hardwareMap.get(BNO055IMU::class.java, "imu")
        imu.initialize(parameters)

        outtakeServo.position = 0.9
        armController.reset()
        targetAngle = restAngle
        targetTicks = restAngle * ticksPerDegree
        armController.targetPosition = targetTicks
        telemetry.addData("STATUS", "Initialized")
        telemetry.update()
    }



    override fun loop() {
        driveControl()
        armControl()
        intakeControl()
        outtakeControl()
        duckControl()
    }

    companion object {
        @JvmStatic var kp = 0.015
        @JvmStatic var ki = 0.0
        @JvmStatic var kd = 0.00075
        @JvmStatic var targetAngle = 0.0
        @JvmStatic var kcos = 0.275
        @JvmStatic var kv = 0.0
        @JvmStatic var depositAngle = 94.0
        @JvmStatic var restAngle = -55.0
        @JvmStatic var sharedAngle = 172.0
        @JvmStatic var sharedAngleAlliance = 174.0
        @JvmStatic var sharedAngleEnemy = 169.0
    }
}