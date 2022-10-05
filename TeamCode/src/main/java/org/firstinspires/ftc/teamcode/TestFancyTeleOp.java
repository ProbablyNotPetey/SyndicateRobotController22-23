package org.firstinspires.ftc.teamcode;

import android.os.Build;

import androidx.annotation.RequiresApi;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "Test FancySyndiOp")
//@Disabled
public class TestFancyTeleOp extends LinearOpMode {

    DcMotor FL;
    DcMotor FR;
    DcMotor BL;
    DcMotor BR;

    @RequiresApi(api = Build.VERSION_CODES.N) // Needed for Functional programming
    @Override
    public void runOpMode() throws InterruptedException {
        int invertedControl = 1;

        // Fix the hardware mapping
        FL = hardwareMap.get(DcMotor.class, "FL"); // 1
        FR = hardwareMap.get(DcMotor.class, "FR"); // 0
        BL = hardwareMap.get(DcMotor.class, "BL"); // 3
        BR = hardwareMap.get(DcMotor.class, "BR"); // 2

        FL.setDirection(DcMotor.Direction.REVERSE); // Test FORWARD vs REVERSE
        FR.setDirection(DcMotor.Direction.FORWARD);
        BL.setDirection(DcMotor.Direction.REVERSE);
        BR.setDirection(DcMotor.Direction.FORWARD);

        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        FL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        telemetry.addData("Status", "Wait for start");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            telemetry.addData("Status", "Running");

            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            double rectTimeCounter = 0;
            double polarTimeCounter = 0;

            while(gamepad1.right_stick_x != 0 ||
                    gamepad1.right_stick_y != 0 ||
                    gamepad1.left_stick_x != 0 ||
                    gamepad1.left_stick_y != 0) {

                sleep(100);
                rectTimeCounter += 100;
                polarTimeCounter += 100;

                if((gamepad1.right_stick_x < 0.1 || gamepad1.right_stick_x > -0.1) &&
                        (gamepad1.right_stick_y < 0.1 || gamepad1.right_stick_y > -0.1))
                    polarTimeCounter = 0;

                if((gamepad1.left_stick_x < 0.1 || gamepad1.left_stick_x > -0.1) &&
                        (gamepad1.left_stick_y < 0.1 || gamepad1.left_stick_y > -0.1))
                    rectTimeCounter = 0;

                double rectCollapse = ConstantFunction.POLY2_COLLAPSE.apply(rectTimeCounter);
                double polarCollapse = ConstantFunction.POLY2_COLLAPSE.apply(polarTimeCounter);

                FL.setPower((y + x) * rectCollapse + rx * polarCollapse);
                FR.setPower((y - x) * rectCollapse - rx * polarCollapse);
                BL.setPower((y - x) * rectCollapse + rx * polarCollapse);
                BR.setPower((y + x) * rectCollapse - rx * polarCollapse);
            }

            telemetry.update();
        }
    }
}


