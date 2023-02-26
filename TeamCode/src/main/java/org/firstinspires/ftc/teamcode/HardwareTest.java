package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.roadrunner.util.Encoder;

@TeleOp
public class HardwareTest extends LinearOpMode {
  private Encoder leftEncoder, rightEncoder, frontEncoder;

  @Override
  public void runOpMode() {
    DcMotor front_right = hardwareMap.dcMotor.get("rightFront");
    DcMotor back_right = hardwareMap.dcMotor.get("rightRear");
    DcMotor front_left = hardwareMap.dcMotor.get("leftFront");
    DcMotor back_left = hardwareMap.dcMotor.get("leftRear");
    DcMotor lift = hardwareMap.dcMotor.get("lift");

    leftEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "leftEncoder"));
    rightEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "rightEncoder"));
    frontEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "frontEncoder"));
    
    front_right.setDirection(DcMotorSimple.Direction.REVERSE);
    back_right.setDirection(DcMotorSimple.Direction.REVERSE);

    lift.setDirection(DcMotorSimple.Direction.REVERSE);
    
    waitForStart();
    
    if (opModeIsActive()) {
      while (opModeIsActive()) {
        front_left.setPower(0.5);
        front_right.setPower(0.5);
        back_left.setPower(0.5);
        back_right.setPower(0.5);

        telemetry.addData("front_left", front_left.getCurrentPosition());
        telemetry.addData("front_right", front_right.getCurrentPosition());
        telemetry.addData("back_left", back_left.getCurrentPosition());
        telemetry.addData("back_right", back_right.getCurrentPosition());
        telemetry.addData("leftEncoder", leftEncoder.getCorrectedVelocity());
        telemetry.addData("rightEncoder", rightEncoder.getCorrectedVelocity());
        telemetry.addData("frontEncoder", frontEncoder.getCorrectedVelocity());
        telemetry.update();
      }
    }
  }
}
