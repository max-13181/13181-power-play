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

        waitForStart();

        setLift(1500, 1);
        claw.setPosition(0.8); // open
        arm.setPosition(0); // forward
        sleep(500);

        int lower = 120;
        int cur_pos = 570;

        setLift(cur_pos, 1);
        cur_pos -= lower;
        claw.setPosition(0.5); // close
        sleep(500);

        // up back release
        placeCone();
        // forward down grab
        grabCone(cur_pos);
        cur_pos -= lower;

        // up back release
        placeCone();
        // forward down grab
        grabCone(cur_pos);
        cur_pos -= lower;

        // up back release
        placeCone();
        // forward down grab
        grabCone(cur_pos);
        cur_pos -= lower;

        // up back release
        placeCone();
        // forward down grab
        grabCone(cur_pos);
        cur_pos -= lower;

        // up back release
        placeCone();
        // forward down grab
        grabCone(cur_pos);

        sleep(10000);
    }

    public void grabCone(int height) {
        arm.setPosition(0); // forward
        sleep(1000);
        setLift(height, 1);
        claw.setPosition(0.5); // close
        sleep(500);
    }

    public void placeCone() {
        lift.setPower(1);
        sleep(500);
        arm.setPosition(1); // backward
        setLift(1750, 1);
        setLift(1500, 0.5);
        claw.setPosition(0.8); // open
        sleep(250);

        // works
        /*
        setLift(lift.getCurrentPosition()+500, 1);
        arm.setPosition(1); // backward
        sleep(1000);
        setLift(1750, 1);
        setLift(1500, 1);
        claw.setPosition(0.8); // open
        sleep(250);
        // */
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
