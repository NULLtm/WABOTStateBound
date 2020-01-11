package org.firstinspires.ftc.teamcode.official;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class WABOTPipeline extends OpenCvPipeline {

    // Our mats
    Mat convert = new Mat(), rect1 = new Mat(), rect2 = new Mat(), rect3 = new Mat();

    // Our translation variables
    final double offsetX = 0, offsetY = 67, scaleX = 10;

    // Threshold for detecting a block
    private final double BRIGHTNESS_THRESHOLD = 65;

    public static double currentBrightness = 0;

    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, convert, Imgproc.COLOR_RGB2Lab);

        int rowStart = (input.rows()-1) / 2 - 10;
        int rowEnd = (input.rows()-1) / 2 + 10;
        int colStart = (input.cols()-1) / 2 - 10;
        int colEnd = (input.cols()-1) / 2 + 10;
        rowEnd+=offsetY;
        rowStart+=offsetY;
        colStart+=offsetX-scaleX;
        colEnd+=offsetX+scaleX;

        rect1 = convert.submat(rowStart, rowEnd, colStart, colEnd);
        rect2 = convert.submat(rowStart, rowEnd, colStart+80, colEnd+80);
        rect3 = convert.submat(rowStart, rowEnd, colStart-80, colEnd-80);

        double leftBlockV = Core.mean(rect3).val[0];
        double middleBlockV = Core.mean(rect1).val[0];
        double rightBlockV = Core.mean(rect2).val[0];

        currentBrightness = leftBlockV;

        if(leftBlockV < rightBlockV && leftBlockV < middleBlockV && leftBlockV < BRIGHTNESS_THRESHOLD){
            Imgproc.putText(input, "Left Block Found", new Point(100, 100), 1, 2, new Scalar(20, 100, 0), 2);
        } else if(rightBlockV < leftBlockV && rightBlockV < middleBlockV && rightBlockV < BRIGHTNESS_THRESHOLD){
            Imgproc.putText(input, "Right Block Found", new Point(100, 100), 1, 2, new Scalar(20, 100, 0), 2);
        } else if(middleBlockV < rightBlockV && middleBlockV < leftBlockV && middleBlockV < BRIGHTNESS_THRESHOLD){
            Imgproc.putText(input, "Middle Block Found", new Point(100, 100), 1, 2, new Scalar(20, 100, 0), 2);
        } else {
            Imgproc.putText(input, "No Block Found", new Point(100, 100), 1, 2, new Scalar(20, 100, 0), 2);
        }

        Imgproc.rectangle(input, new Point(colStart, rowStart), new Point(colEnd, rowEnd), new Scalar(230, 100, 50), 3);
        Imgproc.rectangle(input, new Point(colStart+80, rowStart), new Point(colEnd+80, rowEnd), new Scalar(20, 100, 0), 3);
        Imgproc.rectangle(input, new Point(colStart-80, rowStart), new Point(colEnd-80, rowEnd), new Scalar(20, 100, 0), 3);

        rect1.release();
        rect2.release();
        rect3.release();

        return input;
    }
}
