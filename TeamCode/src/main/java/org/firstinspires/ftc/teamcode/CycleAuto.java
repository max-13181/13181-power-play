package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous
public class CycleAuto extends LinearOpMode {
    DcMotor lift;
    Servo claw;
    Servo arm;

    @Override
    public void runOpMode() throws InterruptedException {
        claw = hardwareMap.servo.get("claw");
        arm = hardwareMap.servo.get("arm");
        lift = hardwareMap.dcMotor.get("lift");

        lift.setDirection(DcMotorSimple.Direction.REVERSE);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(35, -62, Math.toRadians(-90));

        drive.setPoseEstimate(startPose);

        Trajectory traj1 = drive.trajectoryBuilder(startPose)
                .lineTo(new Vector2d(31, -13))
                .build();

        Trajectory traj2 = drive.trajectoryBuilder(traj1.end().plus(new Pose2d(0, 0, Math.toRadians(90))))
                .lineTo(new Vector2d(15, -15))
                .build();

        Trajectory trajToCone = drive.trajectoryBuilder(traj1.end().plus(new Pose2d(0, 0, Math.toRadians(90))))
                .lineToLinearHeading(new Pose2d(50, -13, Math.toRadians(0)))
                .build();

        Trajectory trajToHigh = drive.trajectoryBuilder(trajToCone.end())
                .lineToLinearHeading(new Pose2d(31, -13, Math.toRadians(0)))
                .build();

        claw.setPosition(0.5); // close
        arm.setPosition(0); // forward

        waitForStart();

        drive.followTrajectory(traj1);
        drive.turn(Math.toRadians(90));

        arm.setPosition(0.45); // at angle
        setLift(3900, 0.9);
        claw.setPosition(1); // open
        sleep(750);
        arm.setPosition(0); // forward
        sleep(500);
        setLift(750, 0.9);

        drive.followTrajectory(trajToCone);
        claw.setPosition(0.5); // close
        sleep(750);

        drive.followTrajectory(trajToHigh);
        arm.setPosition(0.45); // at angle
        setLift(3900, 0.9);
        claw.setPosition(1); // open
        sleep(750);
        arm.setPosition(0); // forward
        sleep(500);
        setLift(750, 0.9);

        sleep(10000);
    }

    public void setLift(int pos, double speed) {
        // set target positions
        lift.setTargetPosition(pos);

        // turn on RUN_TO_POSITION
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // set power
        lift.setPower(speed);

        boolean waitCondition = true;

        while (opModeIsActive() && waitCondition) {
            if (!lift.isBusy()) {
                waitCondition = false;
            }
        }

        // stop all motion
        lift.setPower(0.01);

        // turn off RUN_TO_POSITION
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}
