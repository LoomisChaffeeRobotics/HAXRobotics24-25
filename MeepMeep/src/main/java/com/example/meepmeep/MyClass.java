package com.example.meepmeep;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.rowlandhall.meepmeep.MeepMeep;
import org.rowlandhall.meepmeep.roadrunner.DefaultBotBuilder;
import org.rowlandhall.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MyClass {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(23.6, 20, Math.toRadians(180), Math.toRadians(180), 11)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(11.75, -55, Math.toRadians(90)))
                        .forward(20)
                        .forward(5)
                        .back(5)
                        .strafeRight(16)
                        .splineToSplineHeading(new Pose2d(46,-11,Math.PI), 0)
                        .lineToConstantHeading(new Vector2d(48, -56))
                        .lineToConstantHeading(new Vector2d(50, -11))
                        .splineToConstantHeading(new Vector2d(52, -56), Math.toRadians(90))
                        .lineToConstantHeading(new Vector2d(60, -11))
                        .splineToConstantHeading(new Vector2d(63, -56), Math.toRadians(90))


                                .build());

        myBot.setDimensions(17,17);
        meepMeep.setBackground(MeepMeep.Background.FIELD_INTOTHEDEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}