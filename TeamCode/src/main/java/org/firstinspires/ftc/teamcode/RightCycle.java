package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.roadrunner.DriveConstants.MAX_ACCEL;
import static org.firstinspires.ftc.teamcode.roadrunner.DriveConstants.MAX_ANG_VEL;
import static org.firstinspires.ftc.teamcode.roadrunner.DriveConstants.MAX_VEL;
import static org.firstinspires.ftc.teamcode.roadrunner.DriveConstants.TRACK_WIDTH;

import android.annotation.SuppressLint;

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

@Disabled
@Autonomous
public class RightCycle extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Claw claw = new Claw(hardwareMap.servo.get("claw"));
        Arm arm = new Arm(hardwareMap.servo.get("arm"));
        Lift lift = new Lift(hardwareMap.dcMotor.get("lift"));

        AprilTag at = new AprilTag();

        int signal_pos = 1;

        Pose2d startPose = new Pose2d(32, -61.5, Math.toRadians(-90));
        drive.setPoseEstimate(startPose);

        Pose2d highCone = new Pose2d(33, -9, Math.toRadians(0));

        Pose2d correction = new Pose2d(1, 0, Math.toRadians(0));

        TrajectorySequence startToHigh = drive.trajectorySequenceBuilder(startPose)
                .setConstraints(getVel(1.5), getAcc(1.5))
                .addDisplacementMarker(() -> lift.goTo(4000, 1))
                .UNSTABLE_addTemporalMarkerOffset(0.5, arm::mid) // sets arm at an angle
                // goes to high
                .setTangent(Math.toRadians(80))
                .splineToSplineHeading(highCone, Math.toRadians(105))
                // lower lift
                .addDisplacementMarker(() -> {
                    lift.goToSavedPos();
                    lift.lowerSavedPos();
                })
                .build();

        TrajectorySequence cycle = drive.trajectorySequenceBuilder(startToHigh.end().plus(correction))
                // open claw and forward arm
                .UNSTABLE_addTemporalMarkerOffset(0, claw::open)
                .UNSTABLE_addTemporalMarkerOffset(0.2, arm::forward)
                // close claw when at the stack
                .UNSTABLE_addDisplacementMarkerOffset(28, claw::close)
                // drives to stack
                .setTangent(Math.toRadians(-35))
                .splineToConstantHeading(new Pose2d(60, -15, Math.toRadians(0)).vec(), Math.toRadians(0))
                // raises lift
                .UNSTABLE_addTemporalMarkerOffset(0, () -> lift.goTo(4000, 1))
                .waitSeconds(0.4)
                // put arm mid WHILE going to high
                .UNSTABLE_addTemporalMarkerOffset(0.1, arm::mid)
                // goes to high
                .setTangent(Math.toRadians(180))
                .splineToConstantHeading(highCone.vec().plus(correction.vec()), Math.toRadians(180-35))
                // lower lift
                .addDisplacementMarker(() -> {
                    lift.goToSavedPos();
                    lift.lowerSavedPos();
                })
                .build();

        TrajectorySequence park1 = drive.trajectorySequenceBuilder(cycle.end())
                .UNSTABLE_addTemporalMarkerOffset(0, () -> lift.goTo(500, 1))
                .waitSeconds(0.3)
                .UNSTABLE_addTemporalMarkerOffset(0, claw::open)
                .UNSTABLE_addTemporalMarkerOffset(0, arm::forward)
                .setTangent(Math.toRadians(-140))
                .splineToConstantHeading(new Vector2d(15, -12), Math.toRadians(180))
                .build();

        TrajectorySequence park2 = drive.trajectorySequenceBuilder(cycle.end())
                .UNSTABLE_addTemporalMarkerOffset(0, () -> lift.goTo(500, 1))
                .waitSeconds(0.3)
                .UNSTABLE_addTemporalMarkerOffset(0, claw::open)
                .UNSTABLE_addTemporalMarkerOffset(0.2, arm::forward)
                .lineTo(new Vector2d(36, -14))
                .build();

        TrajectorySequence park3 = drive.trajectorySequenceBuilder(cycle.end())
                .UNSTABLE_addTemporalMarkerOffset(0, () -> lift.goTo(500, 1))
                .waitSeconds(0.3)
                .UNSTABLE_addTemporalMarkerOffset(0, claw::open)
                .UNSTABLE_addTemporalMarkerOffset(0.2, arm::forward)
                .setTangent(Math.toRadians(-35))
                .splineToConstantHeading(new Vector2d(59, -14), Math.toRadians(0))
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
            drive.followTrajectorySequence(startToHigh);
            drive.followTrajectorySequence(cycle);
            drive.followTrajectorySequence(cycle);
            drive.followTrajectorySequence(cycle);
            drive.followTrajectorySequence(cycle);
            drive.followTrajectorySequence(cycle);

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

    public static TrajectoryVelocityConstraint getVel(double v) {
        return SampleMecanumDrive.getVelocityConstraint(MAX_VEL * v, MAX_ANG_VEL, TRACK_WIDTH);
    }

    public static TrajectoryAccelerationConstraint getAcc(double v) {
        return SampleMecanumDrive.getAccelerationConstraint(MAX_ACCEL * v);
    }

    class Lift {
        private DcMotor motor;
        private int lower = 100;
        private int current_target = 700;

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
        private double CLAW_CLOSE = 0.53;

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