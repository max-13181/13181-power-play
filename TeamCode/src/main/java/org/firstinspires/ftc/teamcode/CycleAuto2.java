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
public class CycleAuto2 extends LinearOpMode {
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

        Pose2d startPose = new Pose2d(36, -12, Math.toRadians(0));

        drive.setPoseEstimate(startPose);

        Vector2d cone_location = new Vector2d(62, -12);

        Trajectory first_to = drive.trajectoryBuilder(startPose, Math.toRadians(0))
                .lineTo(cone_location)
                .build();

        Trajectory back_to_high = drive.trajectoryBuilder(first_to.end(), Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(33, -8.2), Math.toRadians(160))
                .build();

        Trajectory to_cones = drive.trajectoryBuilder(back_to_high.end(), Math.toRadians(-20))
                .splineToConstantHeading(cone_location, Math.toRadians(0))
                .build();

        claw.setPosition(0.5); // close
        arm.setPosition(0); // forward

        waitForStart();

        claw.setPosition(1); // open

        setLift(650, 1);
        drive.followTrajectory(first_to);
        claw.setPosition(0.5); // close
        sleep(500);
        setLift(1500, 1);

        drive.followTrajectory(back_to_high);
        arm.setPosition(0.45); // angle
        setLift(4000, 1);
        claw.setPosition(1); // open
        sleep(500);
        arm.setPosition(0); // forward
        sleep(200);

        setLift(500, 1);
        drive.followTrajectory(to_cones);
        claw.setPosition(0.5); // close
        sleep(500);
        setLift(1500, 1);

        drive.followTrajectory(back_to_high);
        arm.setPosition(0.45); // angle
        setLift(4000, 1);
        sleep(500);
        claw.setPosition(1); // open
        sleep(500);
        arm.setPosition(0); // forward
        sleep(200);

        setLift(400, 1);
        drive.followTrajectory(to_cones);
        sleep(200);
        claw.setPosition(0.5); // close
        sleep(500);
        setLift(1500, 1);

        drive.followTrajectory(back_to_high);
        arm.setPosition(0.45); // angle
        setLift(4000, 1);
        sleep(500);
        claw.setPosition(1); // open
        sleep(500);
        arm.setPosition(0); // forward
        sleep(200);

        setLift(300, 1);
        drive.followTrajectory(to_cones);
        sleep(200);
        claw.setPosition(0.5); // close

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
