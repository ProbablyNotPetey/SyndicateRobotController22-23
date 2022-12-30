package org.firstinspires.ftc.teamcode.cv;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

/**
 * Simple pipeline that crops the input image around the center,
 * and outputs the parking zone to park in (0, 1, or 2) depending if the input image is Red, Green, or Blue.
 */
public class RGBPipeline extends OpenCvPipeline {
    List<Mat> bgrChannels = new ArrayList<>(3);
    private Mat bgrInput = new Mat();
    private Mat output = new Mat();
    private Mat submat;
    private int parkingZone = -1;

    //TODO Test this extensively. Originally written for Desktop OpenCV so there may be bugs!

    @Override
    public Mat processFrame(Mat input) {

        //INCLUDE THIS LINE ON ROBOT VERSION. EASYOPENCV DOES RGBA, OPENCV DOES BGR!!!!!!!!!!!!
        Imgproc.cvtColor(input, bgrInput, Imgproc.COLOR_RGBA2BGR);

        //DELETE THIS LINE ON ROBOT VERSION.
//        bgrInput = input;


        //Change percent to whatever's best.
        submat = cropCenter(bgrInput, 0.3);

        Core.split(submat, bgrChannels);

        double meanB = Core.mean(bgrChannels.get(0)).val[0];
        double meanG = Core.mean(bgrChannels.get(1)).val[0];
        double meanR = Core.mean(bgrChannels.get(2)).val[0];

        double max = Math.max(meanB, Math.max(meanG, meanR));

        if(max == meanB) parkingZone = 2;
        else if(max == meanG) parkingZone = 1;
        else parkingZone = 0;


        System.out.println("Park in zone " + parkingZone);

        //INCLUDE THIS LINE ON ROBOT VERSION.
        Imgproc.cvtColor(submat, output, Imgproc.COLOR_BGR2RGBA);

        //DELETE THIS LINE ON ROBOT VERSION
//        output = submat;

        return output;
    }

    /**
     * Returns a value from -1 to 2 depending on the parking zone. 0 - 2 corresponds to R, G, and B zones. -1 means val hasn't been calculated yet.
     * @return  zone to park in. -1 means hasn't been calculated yet
     */
    public int getParkingZone() {
        return parkingZone;
    }

    private Mat cropCenter(Mat input, double percent) {
        int centerRow = input.rows()/2;
        int centerCol = input.cols()/2;
        int rows = (int)(input.rows() * percent);
        int cols = (int)(input.cols() * percent);

//        System.out.println("Input dims: " + input.rows() + ", " + input.cols());
//        System.out.println("Corner1: " + new Point((double)(centerCol - (cols/2)), (double)(centerRow - (rows/2))));
//        System.out.println("Corner2: " + new Point((double)(centerCol - (cols/2)) + cols, (double)(centerRow - (rows/2)) + rows));

        return input.submat(new Rect(
                new Point((double)(centerCol - (cols/2)), (double)(centerRow - (rows/2))),
                new Point((double)(centerCol - (cols/2)) + cols, (double)(centerRow - (rows/2)) + rows)
        ));
    }
}
