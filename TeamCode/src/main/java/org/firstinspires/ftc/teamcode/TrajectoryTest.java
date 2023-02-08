package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.roadrunner.DriveConstants.MAX_ACCEL;
import static org.firstinspires.ftc.teamcode.roadrunner.DriveConstants.MAX_ANG_VEL;
import static org.firstinspires.ftc.teamcode.roadrunner.DriveConstants.MAX_VEL;
import static org.firstinspires.ftc.teamcode.roadrunner.DriveConstants.TRACK_WIDTH;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.roadrunner.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;

@Autonomous(group="alpha")
public class TrajectoryTest extends LinearOpMode {

    TrajectoryVelocityConstraint defaultVelocity = SampleMecanumDrive.getVelocityConstraint(MAX_VEL, MAX_ANG_VEL, TRACK_WIDTH);
    TrajectoryAccelerationConstraint defaultAcceleration = SampleMecanumDrive.getAccelerationConstraint(MAX_ACCEL);

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Claw claw = new Claw(hardwareMap.servo.get("claw"));
        Arm arm = new Arm(hardwareMap.servo.get("arm"));
        Lift lift = new Lift(hardwareMap.dcMotor.get("lift"));

        Pose2d startPose = new Pose2d(32, -61.5, Math.toRadians(-90));
        drive.setPoseEstimate(startPose);

        Pose2d highCone = new Pose2d(33, -9, Math.toRadians(0));

        Pose2d correction = new Pose2d(-3, 0, Math.toRadians(0));

        TrajectorySequence startToHighMovementOnly = drive.trajectorySequenceBuilder(startPose)
//                .setConstraints(getVel(2), getAcc(1.5))
                // goes to high
                .setTangent(Math.toRadians(80))
                .splineToSplineHeading(highCone, Math.toRadians(105))
                .build();

        TrajectorySequence cycleTrajMovementOnly = drive.trajectorySequenceBuilder(startToHighMovementOnly.end())
//                .setConstraints(getVel(0.75), getAcc(0.75))
                // drives to stack
                .setTangent(Math.toRadians(-35))
                .splineToConstantHeading(new Pose2d(60, -15, Math.toRadians(0)).vec(), Math.toRadians(0))
                .waitSeconds(0.1)
//                .setConstraints(getVel(1), getAcc(1))
                // goes to high
                .setTangent(Math.toRadians(180))
                .splineToConstantHeading(highCone.vec(), Math.toRadians(180-35))
                .build();

        claw.close();
        arm.forward();

        waitForStart();

        if (!isStopRequested()) {
            drive.followTrajectorySequence(startToHighMovementOnly);
            drive.followTrajectorySequence(cycleTrajMovementOnly);
            drive.followTrajectorySequence(cycleTrajMovementOnly);
            drive.setPoseEstimate(drive.getPoseEstimate().plus(correction));
            drive.followTrajectorySequence(cycleTrajMovementOnly);
            drive.setPoseEstimate(drive.getPoseEstimate().plus(correction));
            drive.followTrajectorySequence(cycleTrajMovementOnly);
            drive.setPoseEstimate(drive.getPoseEstimate().plus(correction));
            drive.followTrajectorySequence(cycleTrajMovementOnly);
        }

        telemetry.addLine("done");
        telemetry.update();

        sleep(10000);
    }

    public static TrajectoryVelocityConstraint getVel(double v) {
        return SampleMecanumDrive.getVelocityConstraint(MAX_VEL * v, MAX_ANG_VEL, TRACK_WIDTH);
    }

    public static TrajectoryAccelerationConstraint getAcc(double v) {
        return SampleMecanumDrive.getAccelerationConstraint(MAX_ACCEL * v);
    }

    class Lift {
        private DcMotor motor;
        private int lower = 100;
        private int current_target = 650;

        public Lift(DcMotor lift_motor) {
            this.motor = lift_motor;
            this.motor.setDirection(DcMotorSimple.Direction.REVERSE);
            this.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        public void goTo(int target, double speed) {
            this.motor.setTargetPosition(target);
            this.motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            this.motor.setPower(speed);
        }

        public void goToSavedPos() {
            this.goTo(this.current_target, 1);
        }

        public void lowerSavedPos() {
            this.current_target -= this.lower;
        }
    }

    class Claw {
        private Servo main;
        private double CLAW_OPEN = 1;
        private double CLAW_CLOSE = 0.5;

        public Claw(Servo claw) {
            this.main = claw;
        }

        public void open() {
            this.main.setPosition(CLAW_OPEN);
        }

        public void close() {
            this.main.setPosition(CLAW_CLOSE);
        }
    }

    class Arm {
        private Servo main;
        private double ARM_FORWARD = 0;
        private double ARM_BACK = 1;
        private double ARM_MID = 0.45;

        public Arm(Servo arm) {
            this.main = arm;
        }

        public void forward() {
            this.main.setPosition(ARM_FORWARD);
        }

        public void back() {
            this.main.setPosition(ARM_BACK);
        }

        public void mid() {
            this.main.setPosition(ARM_MID);
        }

        public void set(double pos) {
            this.main.setPosition(pos);
        }
    }
}