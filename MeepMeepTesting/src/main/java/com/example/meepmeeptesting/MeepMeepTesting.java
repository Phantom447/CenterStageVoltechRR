package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(700);
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setConstraints(100, 100, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                drive.trajectorySequenceBuilder(new Pose2d(-34, 54, 90))
                        .lineToLinearHeading(new Pose2d(-34, 32, Math.toRadians(0)))
                        .waitSeconds(1)
                        .lineToLinearHeading(new Pose2d(-34, 10, Math.toRadians(180)))
                        .lineToLinearHeading(new Pose2d(10,10, Math.toRadians(220)))
                        .lineToLinearHeading(new Pose2d(36,8, Math.toRadians(220)))
                        .lineToLinearHeading(new Pose2d(40,36, Math.toRadians(180)))
                        .waitSeconds(2)

//                        .turn(Math.toRadians(90))
//                        .forward(30).turn(Math.toRadians(90))
//                        .forward(30).turn(Math.toRadians(90))
//                        .forward(30).turn(Math.toRadians(90))
                        .build());
        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_OFFICIAL).setDarkMode(true).setBackgroundAlpha(0.95f).addEntity(myBot).start();
    }
}