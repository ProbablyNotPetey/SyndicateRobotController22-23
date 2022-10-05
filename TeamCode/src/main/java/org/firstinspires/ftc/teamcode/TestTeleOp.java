package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;

@TeleOp(name = "Test SyndiOp")
//@Disabled
public class TestTeleOp extends LinearOpMode {

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

        FR.setDirection(DcMotor.Direction.REVERSE);
        BR.setDirection(DcMotor.Direction.REVERSE);

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

        while (opModeIsActive()) {

            telemetry.addData("Status", "Running");

            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            while(gamepad1.right_stick_x != 0 ||
                    gamepad1.right_stick_y != 0 ||
                    gamepad1.left_stick_x != 0 ||
                    gamepad1.left_stick_y != 0) {
                FR.setPower(y - x - rx);
                FL.setPower(y + x + rx);
                BR.setPower(y + x - rx);
                BL.setPower(y - x + rx);
            }

            telemetry.update();
        }
    }
}


