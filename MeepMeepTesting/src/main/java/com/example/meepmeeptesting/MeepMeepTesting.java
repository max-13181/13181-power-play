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
        drive.setDimensions(15 + 3/8, 15.5);

        // -90 down
        // 90 up
        // 180 right
        // 0 left

        // y is up/down
        // x is left/right

        Pose2d startPose = new Pose2d(31.7, -8.5, Math.toRadians(0));
        drive.setStartPose(startPose);

        // 2
        /*
        RoadRunnerBotEntity path = drive.followTrajectorySequence(traj ->
                traj.trajectorySequenceBuilder(startPose)
                        .lineTo(new Vector2d(36, -12))
                        .build()
        );
        */

        // 3
        /*
        RoadRunnerBotEntity path = drive.followTrajectorySequence(traj ->
                traj.trajectorySequenceBuilder(startPose)
                        // drives to stack
                        .setTangent(Math.toRadians(-35))
                        .splineToConstantHeading(new Vector2d(58, -14), Math.toRadians(0))
                        .build()
        );
        */

        // 1
        /*
        RoadRunnerBotEntity path = drive.followTrajectorySequence(traj ->
                traj.trajectorySequenceBuilder(startPose)
                        // drives to stack
                        .setTangent(Math.toRadians(-140))
                        .splineToConstantHeading(new Vector2d(11, -12), Math.toRadians(180))
                        .build()
        );
        */

        RoadRunnerBotEntity path = drive.followTrajectorySequence(traj ->
                traj.trajectorySequenceBuilder(new Pose2d(11, -12, 0))
                        // drives to stack
                        .setTangent(Math.toRadians(-140))
                        .lineTo(new Vector2d(58, -12))
                        .build()
        );

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(path)
                .start();
    }
}