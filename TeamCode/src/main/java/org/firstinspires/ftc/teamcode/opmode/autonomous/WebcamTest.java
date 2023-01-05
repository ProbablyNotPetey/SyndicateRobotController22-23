package org.firstinspires.ftc.teamcode.opmode.autonomous;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.cv.RGBPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous(name = "Webcam Test")
public class WebcamTest extends LinearOpMode {

    private static final int WEBCAM_WIDTH = 864;
    private static final int WEBCAM_HEIGHT = 480;

    private OpenCvWebcam webcam;

    @Override
    public void runOpMode() throws InterruptedException {

        //Just copy paste this part
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam");
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);

        //Also copy paste this
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(WEBCAM_WIDTH, WEBCAM_HEIGHT, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        //Sets the pipeline
        RGBPipeline pipeline = new RGBPipeline();
        webcam.setPipeline(pipeline);

        waitForStart();

        if(isStopRequested()) return;

        int parkingZone;
//        webcam.stopStreaming();

        while(opModeIsActive()) {

            //THIS TAKES LIKE A SECOND OR TWO BEFORE ITS ACCURATE
            parkingZone = pipeline.getParkingZone();
            telemetry.addData("Position", parkingZone);
            telemetry.update();
        }
    }
}
