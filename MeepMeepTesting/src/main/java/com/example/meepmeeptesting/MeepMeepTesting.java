package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.sun.tools.javac.comp.Todo;

public class MeepMeepTesting {
    public static void main(String[] args) {
        // TODO: If you experience poor performance, enable this flag
        // System.setProperty("sun.java2d.opengl", "true");

        // Declare a MeepMeep instance
        // With a field size of 800 pixels
        MeepMeep mm = new MeepMeep(600)
                // Set field image
                .setBackground(MeepMeep.Background.FIELD_FREIGHT_FRENZY)
                // Set theme
                .setTheme(new ColorSchemeBlueDark())
                // Background opacity from 0-1
                .setBackgroundAlpha(1f)
                // Set constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(70, 50, Math.toRadians(150), Math.toRadians(60), 13.25)
                .setBotDimensions(12.0, 18.0)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(9.0, 62.0, Math.toRadians(90.0)))
                                .setReversed(true)
                                .splineToSplineHeading(new Pose2d(3.5, 37.5, Math.toRadians(50.0)), Math.toRadians(230.0))
                                .setReversed(false)
                                .splineToSplineHeading(new Pose2d(20.5, 63.5, Math.toRadians(0.0)), Math.toRadians(0.0))
                                .splineToSplineHeading(new Pose2d(40.5, 63.5, Math.toRadians(0.0)), Math.toRadians(0.0))
                                .setReversed(true)
                                .splineToSplineHeading(new Pose2d(20.5, 63.5, Math.toRadians(0.0)), Math.toRadians(180.0))
                                .splineToSplineHeading(new Pose2d(3.5, 37.5, Math.toRadians(50.0)), Math.toRadians(230.0))
                                .setReversed(false)
                                .splineToSplineHeading(new Pose2d(20.5, 63.5, Math.toRadians(0.0)), Math.toRadians(0.0))
                                .splineToSplineHeading(new Pose2d(45.5, 63.5, Math.toRadians(0.0)), Math.toRadians(0.0))
                                .setReversed(true)
                                .splineToSplineHeading(new Pose2d(20.5, 63.5, Math.toRadians(0.0)), Math.toRadians(180.0))
                                .splineToSplineHeading(new Pose2d(3.5, 37.5, Math.toRadians(50.0)), Math.toRadians(230.0))
                                .setReversed(false)
                                .splineToSplineHeading(new Pose2d(20.5, 63.5, Math.toRadians(0.0)), Math.toRadians(0.0))
                                .splineToSplineHeading(new Pose2d(50.5, 63.5, Math.toRadians(0.0)), Math.toRadians(0.0))
                                .setReversed(true)
                                .splineToSplineHeading(new Pose2d(20.5, 63.5, Math.toRadians(0.0)), Math.toRadians(180.0))
                                .splineToSplineHeading(new Pose2d(3.5, 37.5, Math.toRadians(50.0)), Math.toRadians(230.0))
                                .setReversed(false)
                                .splineToSplineHeading(new Pose2d(20.5, 63.5, Math.toRadians(0.0)), Math.toRadians(0.0))
                                .splineToSplineHeading(new Pose2d(55.5, 63.5, Math.toRadians(0.0)), Math.toRadians(0.0))
                                .setReversed(true)
                                .splineToSplineHeading(new Pose2d(20.5, 63.5, Math.toRadians(0.0)), Math.toRadians(180.0))
                                .splineToSplineHeading(new Pose2d(3.5, 37.5, Math.toRadians(50.0)), Math.toRadians(230.0))
                                .setReversed(false)
                                .splineToSplineHeading(new Pose2d(20.5, 63.5, Math.toRadians(0.0)), Math.toRadians(0.0))
                                .splineToSplineHeading(new Pose2d(45.5, 63.5, Math.toRadians(0.0)), Math.toRadians(0.0))






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
