package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;

@TeleOp(name = "Supersonics TeleOp: Scuffed Edition")
//@Disabled
public class TestKinematics extends LinearOpMode {

    DcMotor FL , FR , BL , BR , slides , gripper;
    Servo arm;
    BNO055IMU IMU;
    Orientation lastAngles = new Orientation();
    double globalAngle, correction;
    int slidesHeight = 0;
    double armPos = 0.0;
    double gripPos = 0.0;
    boolean bump = false;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "RunOpMode");

        int invertedControl = 1;
        FL = hardwareMap.get(DcMotor.class, "FL");
        FR = hardwareMap.get(DcMotor.class, "FR");
        BL = hardwareMap.get(DcMotor.class, "BL");
        BR = hardwareMap.get(DcMotor.class, "BR");

        slides = hardwareMap.get(DcMotor.class , "slides");
        arm = hardwareMap.get(Servo.class , "arm");
        gripper = hardwareMap.get(DcMotor.class , "gripper");

        // FR.setDirection(DcMotor.Direction.REVERSE);
        // BR.setDirection(DcMotor.Direction.REVERSE);
        slides.setDirection(DcMotor.Direction.FORWARD);

        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        FR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slides.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        gripper.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        gripper.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.RADIANS;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // This is probably built in
        parameters.loggingEnabled      = true; // Causes a vast many issues if not set to true
        parameters.loggingTag          = "IMU";


        IMU = hardwareMap.get(BNO055IMU.class, "imu");

        telemetry.addData("Status", "Wait for start");
        telemetry.update();

        IMU.initialize(parameters);

        while(!IMU.isGyroCalibrated()) {
            sleep(50);
            idle();
        }

        telemetry.addData("Status", "IMU Calibrated");

        long lastTimestamp = System.currentTimeMillis();
        final float sqrt2 = 1.41421356237f;

        telemetry.addData("Status", "Setup");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            if(gamepad1.dpad_right) {
                moveSlides(-2000);
            }else if(gamepad1.dpad_up) {
                moveSlides(-3000);
            }else if(gamepad1.dpad_down) {
                moveSlides(0);
            }
            // Check the encoder, currentposition always = 0
            // Also check if > and < are flipped

            if(gamepad1.x){
                armPos = 0.45;
            }
            else if(gamepad1.a) {
                armPos = 0.0;
            }
            else if(gamepad1.y) {
                armPos = 0.8;
            }

            // Implement setTimeout() Method to multithread slides moving back given enough time
            // to prevent collision with wheel (rn it's manual)
            if(armPos != arm.getPosition() && slides.getCurrentPosition() > -280)
                moveSlides(-280);

            if(slides.getCurrentPosition() < -280)
                arm.setPosition(armPos);

            if(gamepad1.right_trigger > 0.5) {
                gripper.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                gripper.setPower(-0.2);
            }
            else {
                gripper.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                gripper.setTargetPosition(0);
                gripper.setPower(0.2);
                gripper.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                if(gripper.getCurrentPosition() < 20)
                    gripper.setPower(0);
            }


            telemetry.addData("Status", "Running");

            double y = gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            double angle = Math.atan2(y , x) + getAngle();
            double transform = Math.sqrt(x * x + y * y);

            telemetry.addData("transform", transform);

            FL.setPower((transform * ((Math.sin(angle) - Math.cos(angle)) / sqrt2) - rx) / 2);
            FR.setPower((transform * ((-Math.sin(angle) - Math.cos(angle)) / sqrt2) - rx) / 2);
            BL.setPower((transform * ((Math.sin(angle) + Math.cos(angle)) / sqrt2) - rx) / 2);
            BR.setPower((transform * ((-Math.sin(angle) + Math.cos(angle)) / sqrt2) - rx) / 2);

            telemetry.addData("Gripper" , "\nCurPos: " + slides.getCurrentPosition() +
                    "\nArmPos: " + slidesHeight);

            telemetry.addData("FL Power", FL.getPower());
            telemetry.addData("FL Raw" , (Math.sin(angle) - Math.cos(angle)) / sqrt2);

            telemetry.addData("FR Power", FR.getPower());
            telemetry.addData("FR Raw" , (-Math.sin(angle) - Math.cos(angle)) / sqrt2);

            telemetry.addData("BL Power", BL.getPower());
            telemetry.addData("BL Raw" , (Math.sin(angle) + Math.cos(angle)) / sqrt2);

            telemetry.addData("BR Power", BR.getPower());
            telemetry.addData("BR Raw" , (-Math.sin(angle) + Math.cos(angle)) / sqrt2);


            telemetry.addData("Angle", Math.toDegrees(angle));
            telemetry.addData("AngleM", getAngle());

            telemetry.update();
        }
    }

    private void resetAngle()
    {
        lastAngles = IMU.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);

        globalAngle = 0;
    }

    /**
     * Get current cumulative angle rotation from last reset.
     * @return Angle in degrees. + = left, - = right.
     */
    private double getAngle()
    {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the IMU works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = IMU.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }

    private void moveSlides(int pos) {
        slidesHeight = pos;
        slides.setTargetPosition(slidesHeight);
        slides.setPower(slides.getCurrentPosition() > slidesHeight ? -1.0 : 0.5);
        slides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    private void bumpSlides() {
        bump = true;
        if(slides.getCurrentPosition() > -300) {
            slides.setTargetPosition(-300);
            slides.setPower(slides.getCurrentPosition() > slidesHeight ? -1.0 : 0.5);
            slides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
    }

}

