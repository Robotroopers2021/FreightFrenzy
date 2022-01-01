package org.firstinspires.ftc.teamcode.drive.advanced

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import org.firstinspires.ftc.teamcode.advanced.PoseStorage
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive

/**
 * This opmode demonstrates how one would implement field centric control using
 * `SampleMecanumDrive.java`. This file is essentially just `TeleOpDrive.java` with the addition of
 * field centric control. To achieve field centric control, the only modification one needs is to
 * rotate the input vector by the current heading before passing it into the inverse kinematics.
 *
 *
 * See lines 42-57.
 */
@TeleOp(group = "advanced")
class FieldCentricDrive : LinearOpMode() {
    @Throws(InterruptedException::class)
    override fun runOpMode() {
        // Initialize SampleMecanumDrive
        val drive = SampleMecanumDrive(hardwareMap)

        // We want to turn off velocity control for teleop
        // Velocity control per wheel is not necessary outside of motion profiled auto
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER)

        // Retrieve our pose from the PoseStorage.currentPose static field
        // See AutoTransferPose.java for further details
        drive.poseEstimate = PoseStorage.currentPose
        waitForStart()
        if (isStopRequested) return
        while (opModeIsActive() && !isStopRequested) {
            // Read pose
            val poseEstimate = drive.poseEstimate

            // Create a vector from the gamepad x/y inputs
            // Then, rotate that vector by the inverse of that heading
            val (x, y) = Vector2d(
                (-gamepad1.left_stick_y).toDouble(),
                (-gamepad1.left_stick_x).toDouble()
            ).rotated(-poseEstimate.heading)

            // Pass in the rotated input + right stick value for rotation
            // Rotation is not part of the rotated input thus must be passed in separately
            drive.setWeightedDrivePower(
                Pose2d(
                    x,
                    y,
                    (-gamepad1.right_stick_x).toDouble()
                )
            )

            // Update everything. Odometry. Etc.
            drive.update()

            // Print pose to telemetry
            telemetry.addData("x", poseEstimate.x)
            telemetry.addData("y", poseEstimate.y)
            telemetry.addData("heading", poseEstimate.heading)
            telemetry.update()
        }
    }
}