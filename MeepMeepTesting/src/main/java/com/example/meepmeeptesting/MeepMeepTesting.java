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
        drive.setDimensions(15 + 3.0/8.0, 15.5);

        // -90 down
        // 90 up
        // 180 right
        // 0 left

        // y is up/down
        // x is left/right

        RoadRunnerBotEntity path = drive.followTrajectorySequence(traj ->
                traj.trajectorySequenceBuilder(new Pose2d(23, -12, 0))
                        // drives to stack
                        .setTangent(Math.toRadians(-20))
                        .lineToLinearHeading(new Pose2d(58, -12, Math.toRadians(0)))
                        .build()
        );

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(path)
                .start();
    }
}