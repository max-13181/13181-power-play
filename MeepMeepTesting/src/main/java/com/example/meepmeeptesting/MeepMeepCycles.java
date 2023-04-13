package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepCycles {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(650);

        DefaultBotBuilder drive = new DefaultBotBuilder(meepMeep);
        drive.setConstraints(40.9058208784751 * 0.9, 60, 5.184444427490234, Math.toRadians(60), 14.96);
        drive.setDimensions(15.5, 18);

        // -90 down
        // 90 up
        // 180 right
        // 0 left

        // y is up/down
        // x is left/right

        Pose2d startPose = new Pose2d(31.75, -62, Math.toRadians(90));

        Pose2d highCone = new Pose2d(26.5, -9.51, Math.toRadians(0));
        Vector2d stack = new Vector2d(58.5, -9.5);
        drive.setStartPose(startPose);

        RoadRunnerBotEntity path = drive.followTrajectorySequence(traj ->
                traj.trajectorySequenceBuilder(startPose)
                        // goes to high
                        .setTangent(Math.toRadians(40))
                        .splineToSplineHeading(highCone, Math.toRadians(115))

                        .setTangent(Math.toRadians(-20))
                        .splineToConstantHeading(stack, Math.toRadians(0))

                        .setTangent(Math.toRadians(180))
                        .splineToConstantHeading(highCone.vec(), Math.toRadians(180-20))
                        .build()
        );

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(path)
                .start();
    }
}