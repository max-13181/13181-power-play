package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.roadrunner.util.AxisDirection;
import org.firstinspires.ftc.teamcode.roadrunner.util.BNO055IMUUtil;

@Disabled
@TeleOp
public class FieldCentricDrive extends LinearOpMode {
  @Override
  public void runOpMode() {
    double speedMod;

    double claw_pos;
    double arm_pos = 0;
    boolean arm_prev = false;

    DcMotor front_right = hardwareMap.dcMotor.get("rightFront");
    DcMotor back_right = hardwareMap.dcMotor.get("rightRear");
    DcMotor front_left = hardwareMap.dcMotor.get("leftFront");
    DcMotor back_left = hardwareMap.dcMotor.get("leftRear");
    
    front_right.setDirection(DcMotorSimple.Direction.REVERSE);
    back_right.setDirection(DcMotorSimple.Direction.REVERSE);

    DcMotor lift = hardwareMap.dcMotor.get("lift");
    lift.setDirection(DcMotorSimple.Direction.REVERSE);
    lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    Servo claw = hardwareMap.servo.get("claw");
    Servo arm = hardwareMap.servo.get("arm");

    // Retrieve the IMU from the hardware map
    BNO055IMU imu = hardwareMap.get(BNO055IMU.class, "imu");
    BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
    // Technically this is the default, however specifying it is clearer
    parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
    BNO055IMUUtil.remapZAxis(imu, AxisDirection.POS_Y);
    // Without this, data retrieving from the IMU throws an exception
    imu.initialize(parameters);
    
    waitForStart();

    if (isStopRequested()) return;

    while (opModeIsActive()) {

      if (gamepad1.left_bumper) {
        speedMod = 1;
      } else if (gamepad1.left_trigger != 0) {
        speedMod = 0.3;
      } else {
        speedMod = 0.5;
      }

      double y = -gamepad1.left_stick_y * speedMod; // Remember, this is reversed!
      double x = gamepad1.left_stick_x * 1.1 * speedMod; // Counteract imperfect strafing
      double rx = -gamepad1.right_stick_x;

      // Read inverse IMU heading, as the IMU heading is CW positive
      double botHeading = -imu.getAngularOrientation().firstAngle;
      //botHeading = angleFix(botHeading);

      double rotX = x * Math.cos(botHeading) - y * Math.sin(botHeading);
      double rotY = x * Math.sin(botHeading) + y * Math.cos(botHeading);

      // Denominator is the largest motor power (absolute value) or 1
      // This ensures all the powers maintain the same ratio, but only when
      // at least one is out of the range [-1, 1]
      double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
      double frontLeftPower = (rotY + rotX + rx) / denominator;
      double backLeftPower = (rotY - rotX + rx) / denominator;
      double frontRightPower = (rotY - rotX - rx) / denominator;
      double backRightPower = (rotY + rotX - rx) / denominator;

      if (gamepad1.y) {
        imu.initialize(parameters);
      }

      if (gamepad2.y && !arm_prev) {
        if (arm_pos == 1) {
          arm_pos = 0;
        } else {
          arm_pos = 1;
        }
        arm_prev = true;
      } else if (!gamepad2.y) {
        arm_prev = false;
      }
      arm.setPosition(arm_pos);

//    if (gamepad2.left_stick_y != 0) {
//      lift.setPower(-gamepad2.left_stick_y);
//    } else {
//      lift.setPower(0.1);
//    }
      lift.setPower(-gamepad2.left_stick_y);

      if (gamepad2.right_bumper) {
        claw_pos = 0.5; // close
      } else {
        claw_pos = 1;
      }
      claw.setPosition(claw_pos);

      front_left.setPower(frontLeftPower);
      back_left.setPower(backLeftPower);
      front_right.setPower(frontRightPower);
      back_right.setPower(backRightPower);
      telemetry.addData("bot angle", Math.toDegrees(botHeading));
      telemetry.update();
    }
  }
  public double angleFix(double angle) {
    angle = Math.toDegrees(angle);

    // translate to 360
    if (angle < 0) {
      angle = 360 - angle;
    }

    // add 180
    angle += 180;

    // mod 360
    angle = angle % 360;

    // translate back to -180 to 180
    if (angle > 180) {
      angle = -(360 - angle);
    }

    return Math.toRadians(angle);
  }
}
