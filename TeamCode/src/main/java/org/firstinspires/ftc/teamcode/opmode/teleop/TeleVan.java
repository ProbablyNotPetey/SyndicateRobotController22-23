package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;

@TeleOp(name = "TeleVan", group = "Old opmodes")
@Disabled
public class TeleVan extends LinearOpMode {

    DcMotor frontLeft;
    DcMotor frontRight;
    DcMotor backLeft;
    DcMotor backRight;

    DcMotor duckSpin;

    DcMotor arm;
    Servo rightGripper;

    CRServo intake;
    DcMotor starSling;

    @Override
    public void runOpMode() throws InterruptedException {
        int invertedControl = 1;
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        duckSpin = hardwareMap.get(DcMotor.class, "duckSpin");

        double armPower = 0.5;
        boolean hold = false;
        arm = hardwareMap.get(DcMotor.class, "arm");

        rightGripper = hardwareMap.get(Servo.class, "rightGripper");

        intake = hardwareMap.get(CRServo.class, "intake");
        starSling = hardwareMap.get(DcMotor.class, "starSling");

        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        starSling.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("Status", "Wait for start");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            telemetry.addData("Status", "Running");

            if (gamepad2.left_stick_y > 0.2)
            {
                arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                armPower = -0.5;
                hold = false;
            }
            else if (gamepad2.left_stick_y < -0.2)
            {
                arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                armPower = 0.5;
                hold = false;
            }
            else if (!hold)
            {
                arm.setMode(DcMotor.RunMode.RESET_ENCODERS);
                hold = true;
            }
            else
            {
                arm.setTargetPosition(0);
                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                arm.setPower(0.1);
            }

            if (gamepad1.a)
            {
                invertedControl *= 1;
                sleep(500);
            }

            double drive = invertedControl * gamepad1.left_stick_y;
            double strafe = invertedControl * -gamepad1.left_stick_x;
            double spin = gamepad1.right_stick_x * 0.85;

            double flPower = drive + strafe + spin;
            double frPower = drive - strafe - spin;
            double blPower = drive - strafe + spin;
            double brPower = drive + strafe - spin;

            if (gamepad2.a)
            {
                rightGripper.setPosition(.7);
            }
            if (gamepad2.b)
            {
                rightGripper.setPosition(.45);
            }

            frontLeft.setPower(flPower/1.6);
            frontRight.setPower(frPower/1.6);
            backLeft.setPower(blPower/1.6);
            backRight.setPower(brPower/1.6);

            duckSpin.setPower(gamepad2.right_stick_x / 1.4);
            arm.setPower(armPower);

            telemetry.addData("flPower", flPower);
            telemetry.addData("frPower", frPower);
            telemetry.addData("blPower", blPower);
            telemetry.addData("brPower", brPower);

            telemetry.addData("duck", gamepad2.right_stick_x);
            telemetry.addData("arm", armPower);

            telemetry.update();
        }
    }
}

