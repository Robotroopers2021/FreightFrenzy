package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;

public class MeepMeepTesting {
    public static void main(String[] args) {
        // TODO: If you experience poor performance, enable this flag
        // System.setProperty("sun.java2d.opengl", "true");

        // Declare a MeepMeep instance
        // With a field size of 800 pixels
        MeepMeep mm = new MeepMeep(800)
                // Set field image
                .setBackground(MeepMeep.Background.FIELD_FREIGHT_FRENZY)
                // Set theme
                .setTheme(new ColorSchemeRedDark())
                // Background opacity from 0-1
                .setBackgroundAlpha(1f)
                // Set constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(50, 30, Math.toRadians(150), Math.toRadians(60), 13.25)
                .setBotDimensions(11.8, 18.0)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-4, -43, Math.toRadians(290)))
                                .splineToSplineHeading(new Pose2d(43, -67, Math.toRadians(0.0)), Math.toRadians(0))
                                .setReversed(true)
                                .splineToSplineHeading(new Pose2d(-4, -43, Math.toRadians(290)), Math.toRadians(110))
                                .setReversed(false)
//                        drive.trajectorySequenceBuilder(new Pose2d(-3.0, 36.0, Math.toRadians(55.0)))
//                                .splineToSplineHeading( new Pose2d( 15.0, 65.0, Math.toRadians(0.0)), Math.toRadians(0.0))
//                                .splineToSplineHeading( new Pose2d(51.0, 63.0, Math.toRadians(340.0)), Math.toRadians(340.0))

//                                //Initial Deposit Traj
//                                .setReversed(true)
//                                .splineToSplineHeading(new Pose2d(-1, 38, Math.toRadians(55)), Math.toRadians(240))
//                                .setReversed(false)
//
//                                //Moving Back Into Warehouse Traj
//                                .splineToSplineHeading(new Pose2d( 15.0, 63.0, Math.toRadians(0.0)), Math.toRadians(0.0))
//                                .splineToConstantHeading(new Vector2d(44.0, 63.0), Math.toRadians(0.0))
//
//                                //Cycle Deposit Traj
//                                .setReversed(true)
//                                .splineToConstantHeading(new Vector2d(15, 63), Math.toRadians(180))
//                                .splineToSplineHeading(new Pose2d(-1, 38, Math.toRadians(55)), Math.toRadians(240))
//                                .setReversed(false)
//
//                                //Park End Traj
//                                .splineToSplineHeading(new Pose2d( 15.0, 63.0, Math.toRadians(0.0)), Math.toRadians(0.0))
//                                .splineToConstantHeading(new Vector2d(39.0, 63.0), Math.toRadians(0.0))
//                                .setReversed(true)
//                        .splineToSplineHeading(new Pose2d(-9, 37, Math.toRadians(90)), Math.toRadians(4.1))
//                                .setReversed(false)


                .build())
        .start();
    }
}
