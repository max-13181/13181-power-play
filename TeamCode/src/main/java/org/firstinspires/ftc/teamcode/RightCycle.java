package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.roadrunner.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;

@Autonomous
public class RightCycle extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Claw claw = new Claw(hardwareMap.servo.get("claw"));
        Arm arm = new Arm(hardwareMap.servo.get("arm"));
        Lift lift = new Lift(hardwareMap.dcMotor.get("lift"));

        int signal_pos = 2;

        Pose2d startPose = new Pose2d(32, -61.5, Math.toRadians(-90));
        drive.setPoseEstimate(startPose);

        Pose2d highCone = new Pose2d(34, -10, Math.toRadians(0));

        TrajectorySequence startToHigh = drive.trajectorySequenceBuilder(startPose)
                .addDisplacementMarker(() -> {
                    // raises lift
                    lift.goTo(4000, 1);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.5, arm::mid) // sets arm at an angle
                // goes to high
                .setTangent(Math.toRadians(80))
                .splineToSplineHeading(highCone, Math.toRadians(105))
                .UNSTABLE_addTemporalMarkerOffset(0, claw::open)
                .waitSeconds(0.01)
                .build();

        TrajectorySequence cycleTraj = drive.trajectorySequenceBuilder(startToHigh.end())
                // put arm foward and lower lift
                .addTemporalMarker(arm::forward)
                .addTemporalMarker(0.2, () -> {
                    lift.goToSavedPos();
                    lift.lowerSavedPos();
                })
                // drives to stack
                .setTangent(Math.toRadians(-35))
                .splineToSplineHeading(new Pose2d(62, -15, Math.toRadians(0)), Math.toRadians(0))
                // close claw
                .UNSTABLE_addTemporalMarkerOffset(0.1, claw::close)
                .waitSeconds(0.4)
                // raises lift
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    lift.goTo(4000, 1);
                })
                .waitSeconds(0.5)
                // put arm mid WHILE going to high
                .UNSTABLE_addTemporalMarkerOffset(0.2, arm::mid)
                // goes to high
                .setTangent(Math.toRadians(180))
                .splineToSplineHeading(highCone, Math.toRadians(180-35))
                // open claw
                .UNSTABLE_addTemporalMarkerOffset(0, claw::open)
                .waitSeconds(0.1)
                .build();

        TrajectorySequence park1 = drive.trajectorySequenceBuilder(cycleTraj.end())
                .setTangent(Math.toRadians(-140))
                .splineToConstantHeading(new Vector2d(11, -12), Math.toRadians(180))
                .build();

        TrajectorySequence park2 = drive.trajectorySequenceBuilder(cycleTraj.end())
                .lineTo(new Vector2d(36, -12))
                .build();

        TrajectorySequence park3 = drive.trajectorySequenceBuilder(cycleTraj.end())
                .setTangent(Math.toRadians(-35))
                .splineToConstantHeading(new Vector2d(64, -20), Math.toRadians(0))
                .build();

        claw.close();
        arm.forward();

        waitForStart();

        if (!isStopRequested()) {
            drive.followTrajectorySequence(startToHigh);
            drive.followTrajectorySequence(cycleTraj);

            if (signal_pos == 1) {
                drive.followTrajectorySequence(park1);
            } else if (signal_pos == 2) {
                drive.followTrajectorySequence(park2);
            } else if (signal_pos == 3) {
                drive.followTrajectorySequence(park3);
            }
        }

        sleep(10000);
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