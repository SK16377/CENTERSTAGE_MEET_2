package opencv;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

@Disabled
public class blueAudiencePipeline extends OpenCvPipeline {
    //red
    Telemetry telemetry;
    Mat mat = new Mat();
    public enum Location {
        LEFT,
        //middle
        RIGHT,
        //left
        NOT_FOUND
        //left
    }




    private Location location = Location.LEFT;
    static final Rect LEFT_ROI = new Rect(
            new Point(65, 80),
            new Point(135, 160));
    static final Rect RIGHT_ROI = new Rect(
            new Point(200, 85),
            new Point(290, 150));
    static double PERCENT_COLOR_THRESHOLD = 0.2;

    public blueAudiencePipeline(Telemetry t) { telemetry = t; }

    @Override
    public Mat processFrame(Mat input) {

        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);
        //COLOR BGR240,51,100

        //red:
//        Scalar lowHSV = new Scalar(0,100,100);
//        Scalar highHSV = new Scalar(10,255,255);

        Scalar lowHSV = new Scalar(97, 74,72);
        Scalar highHSV = new Scalar(128,255,255);


        Core.inRange(mat, lowHSV, highHSV, mat);


        Mat left = mat.submat(LEFT_ROI);
        Mat right = mat.submat(RIGHT_ROI);

        double leftValue = Core.sumElems(left).val[0] / LEFT_ROI.area() / 255;
        double rightValue = Core.sumElems(right).val[0] / RIGHT_ROI.area() / 255;

        left.release();
        right.release();

        telemetry.addData("Right raw value", (int) Core.sumElems(left).val[0]);
        telemetry.addData("Middle raw value", (int) Core.sumElems(right).val[0]);
        telemetry.addData("middle percentage", Math.round(leftValue * 100) + "%");
        telemetry.addData("right percentage", Math.round(rightValue * 100) + "%");

        boolean stoneLeft = leftValue > PERCENT_COLOR_THRESHOLD;
        boolean stoneRight = rightValue > PERCENT_COLOR_THRESHOLD;

        if (stoneLeft) {
            location = Location.LEFT;
            telemetry.addData("Cube Location", "middle");
        }
        else if (stoneRight) {
            location = Location.RIGHT;
            telemetry.addData("Cube Location", "right");
        }
        else {
            location = Location.NOT_FOUND;
            telemetry.addData("Cube Location", "left");
        }
        telemetry.update();

        Imgproc.cvtColor(mat, mat, Imgproc.COLOR_GRAY2RGB);

        Scalar color = new Scalar(255, 0, 0);
        Scalar colorCenterstage = new Scalar(0, 255, 0);

        Imgproc.rectangle(mat, LEFT_ROI, location == Location.LEFT? colorCenterstage:color);
        Imgproc.rectangle(mat, RIGHT_ROI, location == Location.RIGHT? colorCenterstage:color);

        return mat;
    }

    public Location getLocation() {
        return location;
    }
}