package org.firstinspires.ftc.teamcode.opmode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name="Data Gathering")
//@Disabled
public class DataGathering extends LinearOpMode {

    DcMotorEx FL;
    DcMotorEx FR;
    DcMotorEx BL;
    DcMotorEx BR;

    @Override
    public void runOpMode() throws InterruptedException {
        int invertedControl = 1;
        FL = hardwareMap.get(DcMotorEx.class, "FL");
        FR = hardwareMap.get(DcMotorEx.class, "FR");
        BL = hardwareMap.get(DcMotorEx.class, "BL");
        BR = hardwareMap.get(DcMotorEx.class, "BR");

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

        FL.setPower(1);
        BL.setPower(1);
        FR.setPower(1);
        BR.setPower(1);

        sleep(5000);

        FL.setPower(0);
        BL.setPower(0);
        FR.setPower(0);
        BR.setPower(0);
    }
}
