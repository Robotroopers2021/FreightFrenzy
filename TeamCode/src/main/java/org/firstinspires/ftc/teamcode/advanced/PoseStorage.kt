package org.firstinspires.ftc.teamcode.advanced

import com.acmerobotics.roadrunner.geometry.Pose2d

/**
 * Simple static field serving as a storage medium for the bot's pose.
 * This allows different classes/opmodes to set and read from a central source of truth.
 * A static field allows data to persist between opmodes.
 */
object PoseStorage {
    var currentPose = Pose2d()
}