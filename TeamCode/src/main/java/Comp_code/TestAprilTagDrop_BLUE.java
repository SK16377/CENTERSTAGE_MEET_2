package Comp_code;

import android.graphics.Canvas;
import android.util.Size;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.opencv.core.Mat;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.concurrent.TimeUnit;

import opencv.CenterstageDetector;
import opencv.CenterstageProcessor;

//import com.acmerobotics.roadrunner.geometry.Pose2d;


//import com.acmerobotics.roadrunner.trajectoryBuilder;


@Autonomous(name="APRIL_BLUE_BACK", group="Auto")
public class TestAprilTagDrop_BLUE extends LinearOpMode {
    SampleMecanumDrive drive;
    OpenCvCamera webcam;
    Lift lift;
    Arm arm;
    /**
     * The variable to store our instance of the AprilTag processor.
     */
    private AprilTagProcessor aprilTag;
    private CenterstageProcessor detector;

    /**
     * The variable to store our instance of the vision portal.
     */
    private VisionPortal visionPortal;

    public enum START_POSITION{
        BLUE_LEFT,
        BLUE_RIGHT,
        RED_LEFT,
        RED_RIGHT
    }
    @Override
    public void runOpMode() throws InterruptedException {

        drive = new SampleMecanumDrive(hardwareMap);
        lift = new Lift(hardwareMap, telemetry);
        arm = new Arm(hardwareMap, telemetry);

        initAprilTag(telemetry);

        Pose2d startPose = new Pose2d(16.62, 63.42, Math.toRadians(270.00));
        drive.setPoseEstimate(startPose);
        Trajectory left = drive.trajectoryBuilder(startPose)
                .splineTo(new Vector2d(21.8, 43.81), Math.toRadians(270.00))
                .build();

        Trajectory middle = drive.trajectoryBuilder(startPose)
                .splineTo(new Vector2d(13.22, 38.31), Math.toRadians(270.00))
                .build();


        Trajectory right = drive.trajectoryBuilder(startPose)
                .splineTo(new Vector2d(7.93, 39.13), Math.toRadians(230.00))
                .build();


        Trajectory backup_middle = drive.trajectoryBuilder(middle.end())
                .back(2)
                .build();
        Trajectory drop_middle = drive.trajectoryBuilder(backup_middle.end())
                .lineToLinearHeading(new Pose2d(44.7, 36.75, Math.toRadians(0.00)))
                .build();
        Trajectory deposit_middle = drive.trajectoryBuilder(drop_middle.end())
                .forward(6.5)
                .build();
        Trajectory away_middle = drive.trajectoryBuilder(deposit_middle.end())
                .back(4.5)
                .build();
        Trajectory middle_park = drive.trajectoryBuilder(away_middle.end())
                .strafeLeft(22)
                .build();
        Trajectory backup_left = drive.trajectoryBuilder(left.end())
                .back(3)
                .build();

        Trajectory left_drop = drive.trajectoryBuilder(backup_left.end())
                .lineToLinearHeading(new Pose2d(43.24, 44.09, Math.toRadians(0.00)))
                .build();
        Trajectory deposit_left = drive.trajectoryBuilder(left_drop.end())
                .forward(5.7)
                .build();
        Trajectory away_left = drive.trajectoryBuilder(deposit_left.end())
                .back(4.5)
                .build();
        Trajectory left_park = drive.trajectoryBuilder(away_left.end())
                .strafeLeft(15)
                .build();

        Trajectory backup_right = drive.trajectoryBuilder(left.end())
                .back(5)
                .build();
        Trajectory right_drop = drive.trajectoryBuilder(backup_left.end())
                .lineToLinearHeading(new Pose2d(44.24, 31, Math.toRadians(0.00)))
                .build();

        Trajectory deposit_right = drive.trajectoryBuilder(right_drop.end())
                .forward(7.3)
                .build();
        Trajectory away_right = drive.trajectoryBuilder(deposit_right.end())
                .back(2.5)
                .build();
        Trajectory right_park = drive.trajectoryBuilder(away_right.end())
                .strafeLeft(30)
                .build();

        waitForStart();
        if (isStopRequested()) return;
        CenterstageProcessor.Location location = detector.getLocation();

        visionPortal.setProcessorEnabled(detector, false);
        visionPortal.setProcessorEnabled(aprilTag, true);
        setManualExposure(6, 250);  // Use low exposure time to reduce motion blur

        switch (location) {
            case LEFT: //left
                drive.followTrajectory(left);
                drive.followTrajectory(backup_left);
                drive.followTrajectory(left_drop);
                scoreLow(deposit_left, away_left);
                drive.followTrajectory(left_park);

                //drive.followTrajectory(left_park);
                break;
            case NOT_FOUND: //right
                drive.followTrajectory(right);
                drive.followTrajectory(backup_right);
                drive.followTrajectory(right_drop);
                scoreLow(deposit_right, away_right);
                drive.followTrajectory(right_park);
                break;
            case RIGHT: //middle
                drive.followTrajectory(middle);
                drive.followTrajectory(backup_middle);
                drive.followTrajectory(drop_middle);
                scoreLow(deposit_middle, away_middle);
                drive.followTrajectory(middle_park);
                break;
        }





    }
    public void scoreLow(Trajectory backdrop, Trajectory away){

        arm.goToScoringPos();
        lift.moveToTarget(Lift.LiftPos.LOW_AUTO);

        drive.followTrajectory(backdrop);
        arm.deposit(1);
        drive.followTrajectory(away);
        arm.intakePos();
        lift.moveToTarget(Lift.LiftPos.START);

    }

    /**
     * Initialize the AprilTag processor.
     */
    private void initAprilTag(Telemetry telemetry) {

        // Create the AprilTag processor.
        aprilTag = new AprilTagProcessor.Builder()

                // The following default settings are available to un-comment and edit as needed.
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                //.setDrawTagOutline(true)
                //.setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                //.setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                //.setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)

                // == CAMERA CALIBRATION ==
                // If you do not manually specify calibration parameters, the SDK will attempt
                // to load a predefined calibration for your camera.
                .setLensIntrinsics(622.001, 622.001, 319.803, 241.251)
                // ... these parameters are fx, fy, cx, cy.
                .build();
        detector = new CenterstageProcessor(telemetry);
        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // eg: Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second (default)
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second (default)
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        //aprilTag.setDecimation(3);

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));

        // Choose a camera resolution. Not all cameras support all resolutions.
        builder.setCameraResolution(new Size(320, 240));


        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        builder.enableLiveView(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        //builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        //builder.setAutoStopLiveView(false);

        // Set and enable the processor.
        builder.addProcessors(aprilTag, detector);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        visionPortal.setProcessorEnabled(aprilTag, false);
        visionPortal.setProcessorEnabled(detector, true);

        // Disable or re-enable the aprilTag processor at any time.
        //visionPortal.setProcessorEnabled(aprilTag, true);

    }   // end method initAprilTag()
    /*
        Manually set the camera gain and exposure.
        This can only be called AFTER calling initAprilTag(), and only works for Webcams;
       */
    private void    setManualExposure(int exposureMS, int gain) {
        // Wait for the camera to be open, then use the controls

        if (visionPortal == null) {
            return;
        }

        // Make sure camera is streaming before we try to set the exposure controls
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Camera", "Waiting");
            telemetry.update();
            while (!isStopRequested() && (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                sleep(20);
            }
            telemetry.addData("Camera", "Ready");
            telemetry.update();
        }

        // Set camera controls unless we are stopping.
        if (!isStopRequested())
        {
            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                sleep(50);
            }
            exposureControl.setExposure((long)exposureMS, TimeUnit.MILLISECONDS);
            sleep(20);
            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
            gainControl.setGain(gain);
            sleep(20);
        }
    }
}



