package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
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
                .setTheme(new ColorSchemeRedDark())
                // Background opacity from 0-1
                .setBackgroundAlpha(1f)
                // Set constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(50, 30, Math.toRadians(150), Math.toRadians(60), 13.25)
                .setBotDimensions(12.5, 18.0)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(11, 61.25, Math.toRadians(90.0)))

                                //TODO Initial Deposit Trajectory

                                .setReversed(true)
                                .splineToSplineHeading(new Pose2d(0, 42, Math.toRadians(50)), Math.toRadians(220))
                                .addTemporalMarker(0.2, () -> {

                                        })
                                .addTemporalMarker( 1.5, () -> {

                                })
                                .lineToSplineHeading(new Pose2d(-11, 45, Math.toRadians(90)))
                                .setReversed(false)

                                //TODO Move Into Warehouse 1

                                .splineToSplineHeading(new Pose2d(43, 64.5, Math.toRadians(0.0)), Math.toRadians(0))
                                .setReversed(true)

                                //TODO Cycle Deposit 1

                                .splineToSplineHeading(new Pose2d(-11, 45, Math.toRadians(90)), Math.toRadians(270))
                                .addTemporalMarker(6.5, () -> {

                                })
                                .addTemporalMarker(8.2, () -> {

                                })
                                .addTemporalMarker( 9.5, () -> {

                                })
                                .setReversed(false)

                                //TODO Move Into Warehouse 2

                                .splineToSplineHeading(new Pose2d(46, 64.5, Math.toRadians(0.0)), Math.toRadians(0))
                                .setReversed(true)

                                //TODO Cycle Deposit 2

                                .splineToSplineHeading(new Pose2d(-11, 45, Math.toRadians(90)), Math.toRadians(270))
                                .addTemporalMarker(13.5, () -> {

                                })
                                .addTemporalMarker(14.2, () -> {

                                })
                                .addTemporalMarker( 15.5, () -> {

                                })
                                .setReversed(false)

                                //TODO Move Into Warehouse 3

                                .splineToSplineHeading(new Pose2d(49, 64.5, Math.toRadians(0.0)), Math.toRadians(0))
                                .setReversed(true)

                                //TODO Cycle Deposit 3

                                .splineToSplineHeading(new Pose2d(-11, 45, Math.toRadians(90)), Math.toRadians(270))
                                .addTemporalMarker(19.5, () -> {

                                })
                                .addTemporalMarker(21.2, () -> {

                                })
                                .addTemporalMarker( 22.5, () -> {

                                })
                                .setReversed(false)

                                //TODO Park In Warehouse

                                .splineToSplineHeading(new Pose2d(40, 64.5, Math.toRadians(0.0)), Math.toRadians(0))
                                .setReversed(true)


                                //good cycle spline!!!

//                                .splineToSplineHeading(new Pose2d(43, -65, Math.toRadians(0.0)), Math.toRadians(0))
//                                .setReversed(true)
//                                .splineToSplineHeading(new Pose2d(-10, -43, Math.toRadians(270)), Math.toRadians(90))
//                                .setReversed(false)



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
