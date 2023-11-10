package org.firstinspires.ftc.teamcode.vision;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class ColorDetector extends OpenCvPipeline {


    Telemetry telemetry;
    public ColorDetector(Telemetry t) { telemetry = t; }
    Mat mat = new Mat();
    public enum Location {
        LEFT,
        RIGHT,
        CENTER,
        NOT_FOUND
    }
    private Location location;

    static double PERCENT_COLOR_THRESHOLD = .08;

    /*
     * The core values which define the location and size of the sample regions
     */
    static final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(1,70);
    static final Point REGION1_TOPLEFT_ANCHOR_POINT2 = new Point(213,475);
    static final Point REGION2_TOPLEFT_ANCHOR_POINT = new Point(213.5,70);
    static final Point REGION2_TOPLEFT_ANCHOR_POINT2 = new Point(426,475);
    static final Point REGION3_TOPLEFT_ANCHOR_POINT = new Point(426.5,70);
    static final Point REGION3_TOPLEFT_ANCHOR_POINT2 = new Point(639,475);
    static final int REGION_WIDTH = 200;
    static final int REGION_HEIGHT = 350;

    /*
     * Points which actually define the sample region rectangles, derived from above values
     *
     * Example of how points A and B work to define a rectangle
     *
     *   ------------------------------------
     *   | (0,0) Point A                    |
     *   |                                  |
     *   |                                  |
     *   |                                  |
     *   |                                  |
     *   |                                  |
     *   |                                  |
     *   |                  Point B (70,50) |
     *   ------------------------------------
     *
     */
    Point region1_pointA = new Point(
            REGION1_TOPLEFT_ANCHOR_POINT.x,
            REGION1_TOPLEFT_ANCHOR_POINT.y);
    Point region1_pointB = new Point(
            REGION1_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
            REGION1_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);
    Point region1_pointB_alt = new Point(
            REGION1_TOPLEFT_ANCHOR_POINT2.x,
            REGION1_TOPLEFT_ANCHOR_POINT2.y);
    Point region2_pointA = new Point(
            REGION2_TOPLEFT_ANCHOR_POINT.x,
            REGION2_TOPLEFT_ANCHOR_POINT.y);
    Point region2_pointB = new Point(
            REGION2_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
            REGION2_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);

    Point region2_pointB_alt = new Point(
            REGION2_TOPLEFT_ANCHOR_POINT2.x + REGION_WIDTH,
            REGION2_TOPLEFT_ANCHOR_POINT2.y + REGION_HEIGHT);
    Point region3_pointA = new Point(
            REGION3_TOPLEFT_ANCHOR_POINT.x,
            REGION3_TOPLEFT_ANCHOR_POINT.y);
    Point region3_pointB = new Point(
            REGION3_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
            REGION3_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);
    Point region3_pointB_alt = new Point(
            REGION3_TOPLEFT_ANCHOR_POINT2.x ,
            REGION3_TOPLEFT_ANCHOR_POINT2.y);


    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);
        Scalar lowHSV = new Scalar(110,50,50);
        Scalar highHSV = new Scalar(130,255,255);

        Core.inRange(mat, lowHSV, highHSV, mat);
        Rect LEFT_ROI  = new Rect(region1_pointA, region1_pointB);
        Rect CENTER_ROI = new Rect(region2_pointA, region2_pointB);
        Rect RIGHT_ROI = new Rect(region3_pointA, region3_pointB);
        Mat left = mat.submat(LEFT_ROI);
        Mat center = mat.submat(CENTER_ROI);
        Mat right = mat.submat(RIGHT_ROI);

        double leftValue = Core.sumElems(left).val[0] / LEFT_ROI.area() / 255;
        double centerValue = Core.sumElems(center).val[0] / CENTER_ROI.area() / 255;
        double rightValue = Core.sumElems(right).val[0] / RIGHT_ROI.area() / 255;

        left.release();
        center.release();
        right.release();
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  
//        telemetry.addData("Left raw value",  Core.sumElems(left).val[0]);
//        telemetry.addData("Center raw value",  Core.sumElems(center).val[0]);
//        telemetry.addData("Right raw value",  Core.sumElems(right).val[0]);
        telemetry.addData("Left percentage", Math.round(leftValue * 100) + "%");
        telemetry.addData("Center percentage", Math.round(centerValue * 100) + "%");
        telemetry.addData("Right percentage", Math.round(rightValue * 100) + "%");

        boolean PROPLEFT = leftValue > PERCENT_COLOR_THRESHOLD && leftValue > centerValue && leftValue > rightValue;
        boolean propCENTER = centerValue > PERCENT_COLOR_THRESHOLD && centerValue > rightValue && centerValue > leftValue;
        boolean PROPRIGHT = rightValue > PERCENT_COLOR_THRESHOLD && rightValue > centerValue && rightValue > leftValue;

        if (propCENTER && PROPRIGHT && PROPLEFT) {
            location = Location.NOT_FOUND;
            telemetry.addData("prop Location", "not found");
        }
        else if (propCENTER) {
            location = Location.CENTER;
            telemetry.addData("prop Location", "Center");
        }
        else if(PROPLEFT){
            location = Location.LEFT;
            telemetry.addData("prop Location", "left");
        }else if(PROPRIGHT){
            location = Location.RIGHT;
            telemetry.addData("prop Location", "right");
        } else {
            location = Location.NOT_FOUND;
            telemetry.addData("prop Location", "not Found");
        }
        telemetry.update();

        Imgproc.cvtColor(mat, mat, Imgproc.COLOR_GRAY2RGB);
        Scalar neutral = new Scalar(360,255,50);
        Scalar green  = new Scalar(0, 255, 0);
        Scalar red = new Scalar(8, 81,100);
        Imgproc.rectangle(input, CENTER_ROI,neutral);
        Imgproc.rectangle(input,LEFT_ROI,neutral);
        Imgproc.rectangle(input,RIGHT_ROI,neutral);
        if(location == Location.CENTER){
            Imgproc.rectangle(input, CENTER_ROI,green);
            Imgproc.rectangle(input,LEFT_ROI,red);
            Imgproc.rectangle(input,RIGHT_ROI,red);
        } else if(location == Location.RIGHT){
            Imgproc.rectangle(input, RIGHT_ROI, green);
            Imgproc.rectangle(input,CENTER_ROI,red);
            Imgproc.rectangle(input,LEFT_ROI,red);
        } else if(location == Location.LEFT){
            Imgproc.rectangle(input, LEFT_ROI, green);
            Imgproc.rectangle(input,CENTER_ROI,red);
            Imgproc.rectangle(input,RIGHT_ROI,red);
        }
        return input;
    }

    public Location getLocation() {
        return location;
    }
}