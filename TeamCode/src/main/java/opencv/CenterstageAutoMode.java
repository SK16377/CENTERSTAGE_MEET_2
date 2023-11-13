package opencv;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
//import SampleMec;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name="Centerstage Detector", group="Auto")
@Disabled
public class CenterstageAutoMode extends LinearOpMode {
    OpenCvCamera webcam;
    @Override
    public void runOpMode() throws InterruptedException {
        int cameraMonitorViewId = hardwareMap.appContext
                .getResources().getIdentifier("cameraMonitorViewId",
                        "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        detector_2_ranges detector = new detector_2_ranges(telemetry);
        webcam.setPipeline(detector);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            { webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }
            @Override
            public void onError(int errorCode)
            {
                //This will be called if the camera could not be opened
            }
        }
        );
        Pose2d initPose = new Pose2d(0, 0, 0); // Starting Pose
        Pose2d moveBeyondTrussPose = new Pose2d(0,0,0);
        Pose2d dropPurplePixelPose = new Pose2d(0, 0, 0);
        Pose2d midwayPose1 = new Pose2d(0,0,0);
        Pose2d midwayPose1a = new Pose2d(0,0,0);
        Pose2d intakeStack = new Pose2d(0,0,0);
        Pose2d midwayPose2 = new Pose2d(0,0,0);
        Pose2d dropYellowPixelPose = new Pose2d(0, 0, 0);
        Pose2d parkPose = new Pose2d(0,0, 0);
        double waitSecondsBeforeDrop = 0;
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        waitForStart();
        switch (detector.getLocation()) {
            case LEFT://right
                dropPurplePixelPose = new Pose2d(-26, 8, Math.toRadians(0));
                break;
            case RIGHT://middle
                dropPurplePixelPose = new Pose2d(-30, 3, Math.toRadians(0));
                break;
            case NOT_FOUND://left
                dropPurplePixelPose = new Pose2d(-30, -9, Math.toRadians(-45));
                break;
        }
        midwayPose1 = new Pose2d(-14, 13, Math.toRadians(-45));
        waitSecondsBeforeDrop = 0; //TODO: Adjust time to wait for alliance partner to move from board
        parkPose = new Pose2d(-8, 30, Math.toRadians(-90));
        //break;
        webcam.stopStreaming();
    }
}