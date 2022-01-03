package org.firstinspires.ftc.teamcode.advanced

import com.qualcomm.hardware.bosch.BNO055IMU
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.util.Range
import org.firstinspires.ftc.robotcore.external.navigation.*


/**
 * feel free to change the name or group of your class to better fit your robot
 */
@TeleOp(name = "DriverRelativeControl", group = "tutorial")
class DriverRelativeControls : LinearOpMode() {
    /**
     * make sure to change these motors to your team's preference and configuration
     */
    private var FL: DcMotor? = null
    private var FR: DcMotor? = null
    private var backRight: DcMotor? = null
    private var backLeft: DcMotor? = null
    lateinit var imu: BNO055IMU
    lateinit var angles: Orientation
    lateinit var gravity: Acceleration
    override fun runOpMode() {
        /**
         * you can change the variable names to make more sense
         */
        var driveTurn: Double
        //double driveVertical;
        //double driveHorizontal;
        var gamepadXCoordinate: Double
        var gamepadYCoordinate: Double
        var gamepadHypot: Double
        var gamepadDegree: Double
        var robotDegree: Double
        var movementDegree: Double
        var gamepadXControl: Double
        var gamepadYControl: Double
        /**
         * make sure to change this to how your robot is configured
         */
        FL = hardwareMap.dcMotor["FL"]
        FR = hardwareMap.dcMotor["FR"]
        BR = hardwareMap.dcMotor["BR"]
        BL = hardwareMap.dcMotor["BL"]

        //might need to change the motors being reversed
        FR.setDirection(DcMotorSimple.Direction.REVERSE)
        BR.setDirection(DcMotorSimple.Direction.REVERSE)
        val parameters = BNO055IMU.Parameters()
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"
        parameters.loggingEnabled = true
        parameters.loggingTag = "IMU"
        parameters.accelerationIntegrationAlgorithm = JustLoggingAccelerationIntegrator()
        /**
         * make sure you've configured your imu properly and with the correct device name
         */
        imu = hardwareMap.get(BNO055IMU::class.java, "imu")
        imu.initialize(parameters)
        composeTelemetry()
        waitForStart()
        imu.startAccelerationIntegration(Position(), Velocity(), 1000)
        while (opModeIsActive()) {
            driveTurn = -gamepad1.left_stick_x.toDouble()
            //driveVertical = -gamepad1.right_stick_y;
            //driveHorizontal = gamepad1.right_stick_x;
            gamepadXCoordinate =
                gamepad1.right_stick_x.toDouble() //this simply gives our x value relative to the driver
            gamepadYCoordinate =
                -gamepad1.right_stick_y.toDouble() //this simply gives our y vaue relative to the driver
            gamepadHypot = Range.clip(Math.hypot(gamepadXCoordinate, gamepadYCoordinate), 0.0, 1.0)
            //finds just how much power to give the robot based on how much x and y given by gamepad
            //range.clip helps us keep our power within positive 1
            // also helps set maximum possible value of 1/sqrt(2) for x and y controls if at a 45 degree angle (which yields greatest possible value for y+x)
            gamepadDegree = Math.atan2(gamepadYCoordinate, gamepadXCoordinate)
            //the inverse tangent of opposite/adjacent gives us our gamepad degree
            robotDegree = angle
            //gives us the angle our robot is at
            movementDegree = gamepadDegree - robotDegree
            //adjust the angle we need to move at by finding needed movement degree based on gamepad and robot angles
            gamepadXControl = Math.cos(Math.toRadians(movementDegree)) * gamepadHypot
            //by finding the adjacent side, we can get our needed x value to power our motors
            gamepadYControl = Math.sin(Math.toRadians(movementDegree)) * gamepadHypot
            //by finding the opposite side, we can get our needed y value to power our motors
            /**
             * again, make sure you've changed the motor names and variables to fit your team
             */

            //by mulitplying the gamepadYControl and gamepadXControl by their respective absolute values, we can guarantee that our motor powers will not exceed 1 without any driveTurn
            //since we've maxed out our hypot at 1, the greatest possible value of x+y is (1/sqrt(2)) + (1/sqrt(2)) = sqrt(2)
            //since (1/sqrt(2))^2 = 1/2 = .5, we know that we will not exceed a power of 1 (with no turn), giving us more precision for our driving
            FR.setPower(
                gamepadYControl * Math.abs(gamepadYControl) - gamepadXControl * Math.abs(
                    gamepadXControl
                ) + driveTurn
            )
            backRight.setPower(
                gamepadYControl * Math.abs(gamepadYControl) + gamepadXControl * Math.abs(
                    gamepadXControl
                ) + driveTurn
            )
            frontLeft.setPower(
                gamepadYControl * Math.abs(gamepadYControl) + gamepadXControl * Math.abs(
                    gamepadXControl
                ) - driveTurn
            )
            backLeft.setPower(
                gamepadYControl * Math.abs(gamepadYControl) - gamepadXControl * Math.abs(
                    gamepadXControl
                ) - driveTurn
            )

            /*frontRight.setPower(driveVertical - driveHorizontal + driveTurn);
            backRight.setPower(driveVertical + driveHorizontal + driveTurn);
            frontLeft.setPower(driveVertical + driveHorizontal - driveTurn);
            backLeft.setPower(driveVertical - driveHorizontal - driveTurn);*/
        }
        telemetry.update()
    }

    fun composeTelemetry() {
        telemetry.addAction {
            angles = imu!!.getAngularOrientation(
                AxesReference.INTRINSIC,
                AxesOrder.ZYX,
                AngleUnit.DEGREES
            )
            gravity = imu!!.gravity
        }
    }

    //allows us to quickly get our z angle
    val angle: Double
        get() = imu!!.getAngularOrientation(
            AxesReference.INTRINSIC,
            AxesOrder.ZYX,
            AngleUnit.DEGREES
        ).firstAngle.toDouble()
}