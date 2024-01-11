package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);


        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(73.17330064499293, 73.17330064499293, Math.toRadians(163.999986349878), Math.toRadians(163.999986349878), 17.9)
                .followTrajectorySequence(drive ->
                        //right spike mark
                        drive.trajectorySequenceBuilder(new Pose2d(11, 61, Math.toRadians(-90)))





                                .splineToConstantHeading(new Vector2d(22, 37), Math.toRadians(0))

                                .splineTo(new Vector2d(45, 29), Math.toRadians(0))

                                .turn(Math.toRadians(90))

                                //raise arm and close claw

                                .turn(Math.toRadians(180))

                                .lineTo(new Vector2d(-24, 36))





                                //higher arm, open claw, lower arm, close claw


//
                                .build()
                );
        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}