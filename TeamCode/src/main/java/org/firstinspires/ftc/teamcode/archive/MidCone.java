package org.firstinspires.ftc.teamcode.archive;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.roadrunner.SampleMecanumDrive;

@Disabled
@Autonomous()
public class MidCone extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Servo claw = hardwareMap.servo.get("claw");
        Servo arm = hardwareMap.servo.get("arm");

        DcMotor lift = hardwareMap.dcMotor.get("lift");
        lift.setDirection(DcMotorSimple.Direction.REVERSE);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Trajectory traj1 = drive.trajectoryBuilder(new Pose2d())
                .forward(48)
                .build();

        Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
                .strafeLeft(65)
                .build();

        Trajectory traj3 = drive.trajectoryBuilder(traj2.end())
                .forward(10)
                .build();

        Trajectory traj4 = drive.trajectoryBuilder(traj3.end())
                .back(10)
                .build();

        Trajectory traj5 = drive.trajectoryBuilder(traj4.end())
                .strafeRight(20)
                .build();

        arm.setPosition(0);

        waitForStart();

        claw.setPosition(0.5);

        drive.followTrajectory(traj1); // drives to mid
        drive.followTrajectory(traj2); // drives home
        claw.setPosition(1); // opens claw
        drive.followTrajectory(traj3); // pushes
        drive.followTrajectory(traj4); // pushes
        drive.followTrajectory(traj5); // drives to the tile
    }
}
