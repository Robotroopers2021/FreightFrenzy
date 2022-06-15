package org.firstinspires.ftc.teamcode.neilisgoodteacher

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.noahbres.meepmeep.MeepMeep
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder
import com.noahbres.meepmeep.roadrunner.DriveShim


object MeepMeep {
    @JvmStatic
    fun main(args: Array<String>) {
        val meepMeep = MeepMeep(600)
        val myBot = DefaultBotBuilder(meepMeep) // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
            .setConstraints(60.0, 40.0, Math.toRadians(80.0), Math.toRadians(30.0), 13.25    )
            .setDimensions(12.5, 18.0)
            .followTrajectorySequence { drive: DriveShim ->
                drive.trajectorySequenceBuilder(Pose2d(6.25, 61.5, Math.toRadians(90.0)))
                    .setReversed(true)
                    .splineToSplineHeading(Pose2d(1.0, 34.0, Math.toRadians(40.0)), Math.toRadians(240.0))
                    .build()
            }
        meepMeep.setBackground(MeepMeep.Background.FIELD_FREIGHTFRENZY_ADI_DARK)
            .setDarkMode(true)
            .setBackgroundAlpha(0.95f)
            .addEntity(myBot)
            .start()
    }
}