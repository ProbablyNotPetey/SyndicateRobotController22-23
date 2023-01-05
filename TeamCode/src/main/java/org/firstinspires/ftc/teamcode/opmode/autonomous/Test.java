package org.firstinspires.ftc.teamcode.opmode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;

@Autonomous(name = "Test")
//@Disabled
public class Test extends LinearOpMode {

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

        arm.setPower(0.35);
        sleep(2100);
        arm.setPower(0);
        sleep(3000);
    }
}

