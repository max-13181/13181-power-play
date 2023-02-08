package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class LiftTest extends LinearOpMode {
    DcMotor lift;

    @Override
    public void runOpMode() throws InterruptedException {
        lift = hardwareMap.dcMotor.get("lift");
        lift.setDirection(DcMotorSimple.Direction.REVERSE);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // turn on RUN_TO_POSITION
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("lift pos", lift.getCurrentPosition());
            telemetry.update();
        }
    }

    class Lift {
        private DcMotor main;
        private int pos;
        private double speed;

        public Lift(DcMotor lift) {
            // Beep boop this is the the constructor for the lift
            // Assume this sets up the lift hardware
            this.main = lift;
            this.main.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        public void Set(int pos, double speed) {
            this.pos = pos;
            this.speed = speed;
        }

        public void update() {

            if (Math.abs(this.main.getCurrentPosition() - this.pos) > 100) {

            }

            // set target positions
            this.main.setTargetPosition(this.pos);

            // set power
            lift.setPower(speed);

            boolean waitCondition = true;

            while (opModeIsActive() && waitCondition) {
                if (!lift.isBusy()) {
                    waitCondition = false;
                }
            }

            // stop all motion
            lift.setPower(0);

            // turn off RUN_TO_POSITION
            lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }
}
