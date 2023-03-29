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

        Pose2d startPose = new Pose2d(31.75, -62, Math.toRadians(-90));
        drive.setStartPose(startPose);

        Pose2d highCone = new Pose2d(-32, -8.3, Math.toRadians(0));

        RoadRunnerBotEntity path = drive.followTrajectorySequence(traj ->
                traj.trajectorySequenceBuilder(startPose)
                        .waitSeconds(10)
                        // goes to high
                        .setTangent(Math.toRadians(63))
                        .splineToSplineHeading(highCone, Math.toRadians(70))
//                        .setTangent(Math.toRadians(-160))
//                        .splineToSplineHeading(new Pose2d(-60, -15, Math.toRadians(0)), Math.toRadians(180))
                        .build()
        );

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(path)
                .start();
    }
}