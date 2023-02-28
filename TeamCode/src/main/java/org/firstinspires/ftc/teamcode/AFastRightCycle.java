package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.roadrunner.DriveConstants.MAX_ACCEL;
import static org.firstinspires.ftc.teamcode.roadrunner.DriveConstants.MAX_ANG_VEL;
import static org.firstinspires.ftc.teamcode.roadrunner.DriveConstants.MAX_VEL;
import static org.firstinspires.ftc.teamcode.roadrunner.DriveConstants.TRACK_WIDTH;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.roadrunner.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;

@Config
@Autonomous(group="alpha")
public class AFastRightCycle extends LinearOpMode {

    TrajectoryVelocityConstraint defaultVelocity = SampleMecanumDrive.getVelocityConstraint(MAX_VEL, MAX_ANG_VEL, TRACK_WIDTH);
    TrajectoryAccelerationConstraint defaultAcceleration = SampleMecanumDrive.getAccelerationConstraint(MAX_ACCEL);

    public static int HIGH_HEIGHT = 3700;
    public static int MID_HEIGHT = 1650;
    public static int STACK_START = 625;
    public static int STACK_MID = 1000;
    public static int STACK_INC = 140;

    public static double DROP_DELAY = 1.5;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Claw claw = new Claw(hardwareMap.servo.get("claw"));
        Arm arm = new Arm(hardwareMap.servo.get("arm"));
        Lift lift = new Lift(hardwareMap.dcMotor.get("lift"));

        Pose2d startPose = new Pose2d(31, -63.5 + 4 + 3.0/8.0, Math.toRadians(-90));
        Vector2d stack = new Vector2d(59.3, -14);

        Pose2d highCone =               new Pose2d(31.3, -11.1, Math.toRadians(0));
        Vector2d highConeAfterStack = new Vector2d(31.5, -11.9);

        drive.setPoseEstimate(startPose);

        TrajectorySequence toHigh = drive.trajectorySequenceBuilder(startPose)
                // bumps into the high pole then goes up and down again
                .UNSTABLE_addTemporalMarkerOffset(0.2, () -> {
                    lift.goToSavedPos();
                    lift.lowerSavedPos();
                })

                .UNSTABLE_addTemporalMarkerOffset(0.2, () -> lift.goTo(MID_HEIGHT, 1))
                .UNSTABLE_addTemporalMarkerOffset(2.5, arm::mid) // sets arm at an angle

                .setTangent(Math.toRadians(75))
                .splineToSplineHeading(highCone, Math.toRadians(110))

                .UNSTABLE_addTemporalMarkerOffset(0, () -> lift.goTo(HIGH_HEIGHT, 1))
                .UNSTABLE_addTemporalMarkerOffset(DROP_DELAY, claw::open)
                .waitSeconds(DROP_DELAY)
                .build();

        TrajectorySequence cycle = drive.trajectorySequenceBuilder(toHigh.end())
                .UNSTABLE_addTemporalMarkerOffset(0.2, () -> {
                    lift.goToSavedPos();
                    lift.lowerSavedPos();
                })

                .UNSTABLE_addTemporalMarkerOffset(0.5, arm::forward)
                .UNSTABLE_addTemporalMarkerOffset(0.2, claw::partial_open)

                // close claw when at the stack
                //.UNSTABLE_addDisplacementMarkerOffset(27.5, claw::close)

                // drive to stack
                .setTangent(Math.toRadians(-18))
                .splineToConstantHeading(stack, Math.toRadians(0))

                .UNSTABLE_addTemporalMarkerOffset(0.2, () -> claw.set(0.51))
                .waitSeconds(0.6)

                // raises lift
                .UNSTABLE_addTemporalMarkerOffset(0, () -> lift.goTo(STACK_MID, 1))
                .waitSeconds(0.3)

                .UNSTABLE_addTemporalMarkerOffset(0.2, arm::mid)
                .UNSTABLE_addTemporalMarkerOffset(0.3, () -> lift.goTo(MID_HEIGHT, 1))

                .setTangent(Math.toRadians(180))
                .splineToConstantHeading(highConeAfterStack, Math.toRadians(180+10))

                .UNSTABLE_addTemporalMarkerOffset(0, () -> lift.goTo(HIGH_HEIGHT, 1))
                .UNSTABLE_addTemporalMarkerOffset(DROP_DELAY, claw::open)
                .waitSeconds(DROP_DELAY)
                .build();

        claw.close();
        arm.forward();

        waitForStart();

        if (!isStopRequested()) {
            drive.followTrajectorySequence(toHigh);
            drive.followTrajectorySequence(cycle);
            drive.followTrajectorySequence(cycle);
            drive.followTrajectorySequence(cycle);
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
        private int lower = STACK_INC;
        private int current_target = STACK_START;

        public Lift(DcMotor lift_motor) {
            this.motor = lift_motor;
            this.motor.setDirection(DcMotorSimple.Direction.REVERSE);
            this.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            this.motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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
        private double CLAW_PARTIAL = 0.75;
        private double CLAW_CLOSE = 0.5;

        public Claw(Servo claw) {
            this.main = claw;
        }

        public void open() {
            this.main.setPosition(CLAW_OPEN);
        }

        public void partial_open() {
            this.main.setPosition(CLAW_PARTIAL);
        }

        public void close() {
            this.main.setPosition(CLAW_CLOSE);
        }

        public void set(double pos) {
            this.main.setPosition(pos);
        }
    }

    class Arm {
        private Servo main;
        private double ARM_FORWARD = 0;
        private double ARM_BACK = 1;
        private double ARM_MID = 0.44;

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