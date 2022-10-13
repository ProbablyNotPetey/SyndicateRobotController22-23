package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;

@TeleOp(name = "Test Kinematics")
//@Disabled
public class TestKinematics extends LinearOpMode {

    DcMotor FL;
    DcMotor FR;
    DcMotor BL;
    DcMotor BR;

    @Override
    public void runOpMode() throws InterruptedException {
        int invertedControl = 1;
        FL = hardwareMap.get(DcMotor.class, "FL");
        FR = hardwareMap.get(DcMotor.class, "FR");
        BL = hardwareMap.get(DcMotor.class, "BL");
        BR = hardwareMap.get(DcMotor.class, "BR");

        // FR.setDirection(DcMotor.Direction.REVERSE);
        // BR.setDirection(DcMotor.Direction.REVERSE);

        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        FR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        telemetry.addData("Status", "Wait for start");
        telemetry.update();

        waitForStart();

        long lastTimestamp = System.currentTimeMillis();
        final float sqrt2 = 1.414213f;

        while (opModeIsActive()) {
            sleep(1);
            telemetry.addData("Status", "Running");

            double y = gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            double angle = Math.atan2(y , x);
            double transform = Math.sqrt(Math.sqrt(x * x + y * y) + Math.abs(rx)) / 2;

            telemetry.addData("transform", transform);

            FL.setPower(transform * (Math.sin(angle) - Math.cos(angle)));
            FR.setPower(transform * (-Math.sin(angle) - Math.cos(angle)));
            BL.setPower(transform * (Math.sin(angle) + Math.cos(angle)));
            BR.setPower(transform * (-Math.sin(angle) + Math.cos(angle)));

            telemetry.addData("FL Power", FL.getPower());
            telemetry.addData("FL Raw" , (Math.sin(angle) - Math.cos(angle)) / sqrt2);

            telemetry.addData("FR Power", FR.getPower());
            telemetry.addData("FR Raw" , (-Math.sin(angle) - Math.cos(angle)) / sqrt2);

            telemetry.addData("BL Power", BL.getPower());
            telemetry.addData("BL Raw" , (Math.sin(angle) + Math.cos(angle)) / sqrt2);

            telemetry.addData("BR Power", BR.getPower());
            telemetry.addData("BR Raw" , (-Math.sin(angle) + Math.cos(angle)) / sqrt2);


            telemetry.addData("Angle", Math.toDegrees(Math.atan(gamepad1.left_stick_y / gamepad1.left_stick_x)));
            telemetry.addData("Angle", angle);

            telemetry.update();
        }
    }
}
