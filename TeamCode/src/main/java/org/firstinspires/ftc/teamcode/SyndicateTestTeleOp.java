package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.hardware.DcMotor;


@TeleOp(name = "SyndiOp")
//@Disabled
public class SyndicateTestTeleOp extends LinearOpMode {

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

        while (opModeIsActive()) {

            telemetry.addData("Status", "Running");

            double y = gamepad1.left_stick_y / 3;
            double x = gamepad1.left_stick_x / 3;
            double rx = gamepad1.right_stick_x / 3;

            // FL.setPower(y - x - rx);
            // FR.setPower(y + x + rx);
            // BL.setPower(y + x - rx);
            // BR.setPower(y - x + rx);

            /*
            FL:  y - x - rx
            FR: -y - x - rx
            BL:  y + x - rx
            BR: -y + x - rx
            */
            FL.setPower(3*y - 3*x - 3*rx);
            FR.setPower(3*-y - 3*x - 3*rx);
            BL.setPower(3*y + 3*x - 3*rx);
            BR.setPower(3*-y + 3*x - 3*rx);

            telemetry.addData("FL Power", FL.getPower());
            telemetry.addData("FL Raw" , y + x + rx);

            telemetry.addData("FR Power", FR.getPower());
            telemetry.addData("FR Raw" , y - x - rx);

            telemetry.addData("BL Power", BL.getPower());
            telemetry.addData("BL Raw" , y - x + rx);

            telemetry.addData("BR Power", BR.getPower());
            telemetry.addData("BR Raw" , y + x - rx);

            telemetry.addData("x" , x*3);
            telemetry.addData("x raw" , x);
            telemetry.addData("y" , y*3);
            telemetry.addData("y raw" , y);
            telemetry.addData("Angle", Math.toDegrees(Math.atan(gamepad1.left_stick_y / gamepad1.left_stick_x)));

            telemetry.update();
        }
    }

}
