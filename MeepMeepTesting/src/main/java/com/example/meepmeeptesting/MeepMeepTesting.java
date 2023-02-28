package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import org.jetbrains.annotations.NotNull;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(650);

        DefaultBotBuilder drive = new DefaultBotBuilder(meepMeep);
        drive.setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15);
        drive.setDimensions(14, 14.25);

        // -90 down
        // 90 up
        // 180 right
        // 0 left

        // y is up/down
        // x is left/right

        Pose2d startPose = new Pose2d(31, -63.5 + 4 + 3.0/8.0, Math.toRadians(-90));

        RoadRunnerBotEntity path = drive.followTrajectorySequence(traj ->
                traj.trajectorySequenceBuilder(startPose)
                        .waitSeconds(1)
                        .setTangent(Math.toRadians(180))
                        .splineToConstantHeading(new Vector2d(13, -45), Math.toRadians(90))
                        .splineToSplineHeading(new Pose2d(31.3, -11.1, Math.toRadians(0)), Math.toRadians(0))
                        .waitSeconds(1)
                        .build()
        );

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(path)
                .start();
    }
}