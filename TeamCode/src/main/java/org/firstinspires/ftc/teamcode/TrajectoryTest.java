package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(group="alpha")
public class TrajectoryTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Claw claw = new Claw(hardwareMap.servo.get("claw"));
        Arm arm = new Arm(hardwareMap.servo.get("arm"));
        DcMotor lift = hardwareMap.dcMotor.get("lift");

        lift.setDirection(DcMotorSimple.Direction.REVERSE);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Pose2d startPose = new Pose2d(32, -61.5, Math.toRadians(-90));
        drive.setPoseEstimate(startPose);

        /*
        TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(startPose)
                // comes away from the wall
                .lineTo(new Vector2d(35, -53))
                .addDisplacementMarker(() -> {
                    // raises lift
                    lift.setTargetPosition(4000);
                    lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    lift.setPower(1);
                })
                .UNSTABLE_addTemporalMarkerOffset(1, arm::mid) // sets arm at an angle
                // move to high junction and spins
                .setTangent(Math.toRadians(90))
                .splineToSplineHeading(new Pose2d(31.5, -8.3, Math.toRadians(0)), Math.toRadians(110))
                .addDisplacementMarker(claw::open)
                .waitSeconds(1)
                .addDisplacementMarker(arm::forward)
                .addDisplacementMarker(20, () -> {
                    // lowers lift
                    lift.setTargetPosition(1000);
                    lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    lift.setPower(1);
                })
                // drives to stack
                .setTangent(Math.toRadians(-35))
                .splineToSplineHeading(new Pose2d(60, -14, Math.toRadians(0)), Math.toRadians(0))
                .waitSeconds(5)
                // goes to high
                .setTangent(Math.toRadians(180))
                .splineToSplineHeading(new Pose2d(31.5, -8.3, Math.toRadians(0)), Math.toRadians(180-35))
                .addDisplacementMarker(() -> {
                    // raises lift
                    lift.setTargetPosition(4000);
                    lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    lift.setPower(1);
                })
                .UNSTABLE_addTemporalMarkerOffset(1, arm::mid) // sets arm at an angle
                .build();
        // */

        int lower = 120;
        int cur_pos = 570;

        Pose2d highCone = new Pose2d(31.5, -8.3, Math.toRadians(0));

        TrajectorySequence startToHigh = drive.trajectorySequenceBuilder(startPose)
                // comes away from the wall
                .lineTo(new Vector2d(35, -53))
                .addDisplacementMarker(() -> {
                    // raises lift
                    lift.setTargetPosition(4000);
                    lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    lift.setPower(1);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.5, arm::mid) // sets arm at an angle
                // move to high junction and spins
                .setTangent(Math.toRadians(90))
                .splineToSplineHeading(highCone, Math.toRadians(110))
                .UNSTABLE_addTemporalMarkerOffset(1, claw::open)
                .waitSeconds(2)
                .build();

        TrajectorySequence cycleTraj = drive.trajectorySequenceBuilder(startToHigh.end())
                .addDisplacementMarker(arm::forward)
                .addTemporalMarker(0.2, () -> {
                    // lowers lift
                    lift.setTargetPosition(650);
                    lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    lift.setPower(1);
                })
                // drives to stack
                .setTangent(Math.toRadians(-35))
                .splineToSplineHeading(new Pose2d(62, -15, Math.toRadians(0)), Math.toRadians(0))
                .UNSTABLE_addTemporalMarkerOffset(0, claw::close)
                .waitSeconds(1)
                // raises lift
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    lift.setTargetPosition(4000);
                    lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    lift.setPower(1);
                })
                .waitSeconds(1)
                .UNSTABLE_addTemporalMarkerOffset(0.2, arm::mid)
                // goes to high
                .setTangent(Math.toRadians(180))
                .splineToSplineHeading(highCone, Math.toRadians(180-35))
                .UNSTABLE_addTemporalMarkerOffset(1, claw::open)
                .waitSeconds(2)
                .build();

        claw.close();
        arm.forward();

        waitForStart();

        if (!isStopRequested())
            drive.followTrajectorySequence(startToHigh);
            drive.followTrajectorySequence(cycleTraj);

        sleep(10000);
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