package opencv;

import android.graphics.Canvas;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

//blue
@Disabled
public class CenterstageProcessor implements VisionProcessor {
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
            new Point(30, 80),
            new Point(100, 160));
    static final Rect RIGHT_ROI = new Rect(
            new Point(170, 85),
            new Point(260, 150));
    static double PERCENT_COLOR_THRESHOLD = 0.2;

    public CenterstageProcessor(Telemetry t) { telemetry = t; }

    @Override
    public void init(int width, int height, CameraCalibration calibration) {

    }

    @Override
    public Object processFrame(Mat input, long captureTimenanos) {

        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);
        //COLOR BGR240,51,100

//        Scalar lowHSV = new Scalar(100,100,100);
//        Scalar highHSV = new Scalar(180,255,255);
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
        telemetry.addData("Right percentage", Math.round(leftValue * 100) + "%");
        telemetry.addData("Middle percentage", Math.round(rightValue * 100) + "%");

        boolean stoneLeft = leftValue > PERCENT_COLOR_THRESHOLD;
        boolean stoneRight = rightValue > PERCENT_COLOR_THRESHOLD;

        if (stoneLeft) {
            location = Location.LEFT;
            telemetry.addData("Cube Location", "left");
        }
        else if (stoneRight) {
            location = Location.RIGHT;
            telemetry.addData("Cube Location", "middle");
        }
        else {
            location = Location.NOT_FOUND;
            telemetry.addData("Cube Location", "right");
        }
        telemetry.update();

        Imgproc.cvtColor(mat, mat, Imgproc.COLOR_GRAY2RGB);

        Scalar red = new Scalar(255, 0, 0);
        Scalar green = new Scalar(0, 255, 0);

        Imgproc.rectangle(mat, LEFT_ROI, location == Location.LEFT? green:red);//middle
        Imgproc.rectangle(mat, RIGHT_ROI, location == Location.RIGHT? green:red);//right

        mat.copyTo(input);
        return null;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {

    }

    public Location getLocation() {
        return location;
    }
}