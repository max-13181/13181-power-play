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
public class Cone extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Servo claw = hardwareMap.servo.get("claw");
        Servo arm = hardwareMap.servo.get("arm");

        DcMotor lift = hardwareMap.dcMotor.get("lift");
        lift.setDirection(DcMotorSimple.Direction.REVERSE);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Trajectory traj1 = drive.trajectoryBuilder(new Pose2d())
                .strafeLeft(35)
                .build();

        Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
                .forward(35)
                .build();

        Trajectory traj3 = drive.trajectoryBuilder(traj2.end())
                .back(35)
                .build();

        Trajectory traj4 = drive.trajectoryBuilder(traj3.end())
                .strafeRight(60)
                .build();

        arm.setPosition(0);

        waitForStart();


        arm.setPosition(0.5); // close
        sleep(500);
        // lift it up
        lift.setPower(1);
        sleep(1000);
        lift.setPower(0);

        drive.followTrajectory(traj1);
        drive.followTrajectory(traj2);

        arm.setPosition(1); // open
        sleep(1000);

        drive.followTrajectory(traj3);
        drive.followTrajectory(traj4);
    }
}
