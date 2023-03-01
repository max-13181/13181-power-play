package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.roadrunner.DriveConstants.MAX_ACCEL;
import static org.firstinspires.ftc.teamcode.roadrunner.DriveConstants.MAX_ANG_VEL;
import static org.firstinspires.ftc.teamcode.roadrunner.DriveConstants.MAX_VEL;
import static org.firstinspires.ftc.teamcode.roadrunner.DriveConstants.TRACK_WIDTH;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.roadrunner.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Config
@Autonomous(group="alpha")
public class LeftCycle extends LinearOpMode {

    public static int HIGH_HEIGHT = 3800;
    public static int MID_HEIGHT = 2000 ;

    public static double ARM_ANGLE = 0.23;
    public static double ARM_ANGLE_AFTER_STACK = 0.23;

    final public static double START_TAN = 60;

    final public static double MID_X = -35;
    final public static double MID_Y = -35;
    final public static double MID_TAN = 112;

    public static double HIGH_X = -27;
    public static double HIGH_Y = -12.3;
    public static double HIGH_TAN = 20;

    public static int STACK_START = 465;
    public static int STACK_MID = 1000;
    final public static int STACK_INC = 140;

    public static double STACK_X = -53.8;
    public static double STACK_Y = -13.7;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Claw claw = new Claw(hardwareMap.servo.get("claw"));
        Arm arm = new Arm(hardwareMap.servo.get("arm"));
        Lift lift = new Lift(hardwareMap.dcMotor.get("lift"));

        AprilTag at = new AprilTag();

        int signal_pos = 0;

        Pose2d startPose = new Pose2d(-39, -63.5 + 3.6, Math.toRadians(-90));
        Vector2d stack = new Vector2d(STACK_X, STACK_Y);

        Pose2d midPoint = new Pose2d(MID_X, MID_Y, Math.toRadians(-90));

        Pose2d highCone = new Pose2d(HIGH_X, HIGH_Y, Math.toRadians(0));
        Vector2d highConeAfterStack = new Vector2d(31.7, -12.2);

        drive.setPoseEstimate(startPose);

        TrajectorySequence toHigh = drive.trajectorySequenceBuilder(startPose)
                .UNSTABLE_addTemporalMarkerOffset(0.2, () -> lift.goTo(MID_HEIGHT, 1))
                .UNSTABLE_addTemporalMarkerOffset(3, arm::mid) // sets arm at an angle

                .setTangent(Math.toRadians(START_TAN))
                .splineToConstantHeading(midPoint.vec(), Math.toRadians(MID_TAN))
                //.splineToConstantHeading(highCone.vec(), Math.toRadians(HIGH_TAN))
                //.turn(Math.toRadians(90))
                .splineToSplineHeading(highCone, Math.toRadians(HIGH_TAN))

                .UNSTABLE_addTemporalMarkerOffset(0, () -> lift.goTo(HIGH_HEIGHT, 1))
                .UNSTABLE_addTemporalMarkerOffset(5, claw::open)
                .waitSeconds(5)
                .build();

        TrajectorySequence cycle = drive.trajectorySequenceBuilder(toHigh.end())
                .UNSTABLE_addTemporalMarkerOffset(0.2, () -> {
                    lift.goToSavedPos();
                    lift.lowerSavedPos();
                })

                .UNSTABLE_addTemporalMarkerOffset(0.2, claw::partial_open)
                .UNSTABLE_addTemporalMarkerOffset(0.5, arm::back)

                // drive to stack
                .setTangent(Math.toRadians(198))
                .splineToConstantHeading(stack, Math.toRadians(180))

                .UNSTABLE_addTemporalMarkerOffset(0.2, () -> claw.set(0.51))
                .waitSeconds(1)

                // raises lift
                .UNSTABLE_addTemporalMarkerOffset(0, () -> lift.goTo(STACK_MID, 1))
                .waitSeconds(0.3)

                .UNSTABLE_addTemporalMarkerOffset(0.6, () -> arm.set(ARM_ANGLE_AFTER_STACK))

                .UNSTABLE_addTemporalMarkerOffset(1, () -> lift.goTo(HIGH_HEIGHT, 1))
                .setTangent(Math.toRadians(0))
                .splineToConstantHeading(highCone.vec(), Math.toRadians(10))

                .UNSTABLE_addTemporalMarkerOffset(1.5, claw::open)
                .waitSeconds(1.5)
                .build();

        TrajectorySequence park1 = drive.trajectorySequenceBuilder(cycle.end())
                .waitSeconds(0.2)
                .UNSTABLE_addTemporalMarkerOffset(1, claw::open)
                .UNSTABLE_addTemporalMarkerOffset(1, arm::forward)

                .setTangent(Math.toRadians(198))
                .splineToConstantHeading(stack.plus(new Vector2d(-3, 0)), Math.toRadians(180))
                .build();

        TrajectorySequence park2 = drive.trajectorySequenceBuilder(cycle.end())
                .waitSeconds(0.2)
                .UNSTABLE_addTemporalMarkerOffset(1, claw::open)
                .UNSTABLE_addTemporalMarkerOffset(1, arm::forward)

                .lineTo(new Vector2d(-36, -14))
                .build();

        TrajectorySequence park3 = drive.trajectorySequenceBuilder(cycle.end())
                .waitSeconds(0.2)
                .UNSTABLE_addTemporalMarkerOffset(1, claw::open)
                .UNSTABLE_addTemporalMarkerOffset(1, arm::forward)

                .lineTo(new Vector2d(-11, -12))
                .build();

        claw.close();
        arm.forward();

        while (!isStarted() && !isStopRequested()) {
            at.detect();
        }

        if (at.tagOfInterest != null) {
            signal_pos = at.tagOfInterest.id;
        }

        if (!isStopRequested()) {
            drive.followTrajectorySequence(toHigh);
            drive.followTrajectorySequence(cycle);
            drive.followTrajectorySequence(cycle);
//            drive.followTrajectorySequence(cycle);

            if (signal_pos == 1) {
                drive.followTrajectorySequence(park1);
            } else if (signal_pos == 2) {
                drive.followTrajectorySequence(park2);
            } else if (signal_pos == 3) {
                drive.followTrajectorySequence(park3);
            }
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
        private double ARM_MID = ARM_ANGLE;

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

    class AprilTag {
        OpenCvCamera camera;
        org.firstinspires.ftc.teamcode.AprilTagDetectionPipeline aprilTagDetectionPipeline;

        static final double FEET_PER_METER = 3.28084;

        // Lens intrinsics
        // UNITS ARE PIXELS
        // NOTE: this calibration is for the C920 webcam at 800x448.
        // You will need to do your own calibration for other configurations!
        double fx = 578.272;
        double fy = 578.272;
        double cx = 402.145;
        double cy = 221.506;

        // UNITS ARE METERS
        double tagsize = 0.166;

        AprilTagDetection tagOfInterest = null;

        public AprilTag() {
            int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
            camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
            aprilTagDetectionPipeline = new org.firstinspires.ftc.teamcode.AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

            camera.setPipeline(aprilTagDetectionPipeline);
            camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
            {
                @Override
                public void onOpened()
                {
                    camera.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
                }

                @Override
                public void onError(int errorCode)
                {

                }
            });
        }

        public void detect() {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if(currentDetections.size() != 0) {
                boolean tagFound = false;

                for(AprilTagDetection tag : currentDetections) {
                    if(tag.id == 1 || tag.id == 2 || tag.id == 3) {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }

                if(tagFound) {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                }
                else {
                    telemetry.addLine("Don't see tag of interest :(");

                    if(tagOfInterest == null) {
                        telemetry.addLine("(The tag has never been seen)");
                    } else {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }

            } else {
                telemetry.addLine("Don't see tag of interest :(");

                if(tagOfInterest == null) {
                    telemetry.addLine("(The tag has never been seen)");
                } else {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }

            }

            telemetry.update();
            sleep(20);
        }

        @SuppressLint("DefaultLocale")
        void tagToTelemetry(AprilTagDetection detection)
        {
            telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
            telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
            telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
            telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
            telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
            telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
            telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
        }
    }
}