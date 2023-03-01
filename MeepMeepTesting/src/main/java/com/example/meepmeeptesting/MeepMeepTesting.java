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
        drive.setDimensions(14, 14.25); // without boots
        drive.setDimensions(15 + 3.0/8.0, 15.25);

        // -90 down
        // 90 up
        // 180 right
        // 0 left

        // y is up/down
        // x is left/right

        Pose2d startPose = new Pose2d(-39, -63.5 + 3.6, Math.toRadians(-90));

        Pose2d midPoint = new Pose2d(-35, -35, Math.toRadians(-90));

        Pose2d highCone = new Pose2d(-31, -11.1, Math.toRadians(0));

        Vector2d stack = new Vector2d(-59.3, -14);

        RoadRunnerBotEntity path = drive.followTrajectorySequence(traj ->
                traj.trajectorySequenceBuilder(startPose)
                        .waitSeconds(1)
                        .setTangent(Math.toRadians(50))
                        .splineToConstantHeading(midPoint.vec(), Math.toRadians(100))
                        .splineToSplineHeading(highCone, Math.toRadians(70))
                        //.waitSeconds(1)

                        .setTangent(Math.toRadians(0))
                        .lineTo(new Vector2d(-11, -12))
                        .build()
        );

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(path)
                .start();
    }
}