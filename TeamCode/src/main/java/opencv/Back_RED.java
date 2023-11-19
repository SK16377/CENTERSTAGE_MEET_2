package opencv;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import Comp_code.Lift;

//import com.acmerobotics.roadrunner.geometry.Pose2d;


//import com.acmerobotics.roadrunner.trajectoryBuilder;


@Autonomous(name="Red_PARK", group="Auto")
public class Back_RED extends LinearOpMode {
    OpenCvCamera webcam;
    Lift lift;
    public enum START_POSITION{
        BLUE_LEFT,
        BLUE_RIGHT,
        RED_LEFT,
        RED_RIGHT
    }
    @Override
    public void runOpMode() throws InterruptedException {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        lift = new Lift(hardwareMap, telemetry);
        int cameraMonitorViewId = hardwareMap.appContext
                .getResources().getIdentifier("cameraMonitorViewId",
                        "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        detector_2_ranges detector = new detector_2_ranges(telemetry);
        //START_POSITION position = new CenterstageDetector(telemetry);
        webcam.setPipeline(detector);
        //private detector.getLocation location = detector.getLocation().LEFT;
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
                                         @Override
                                         public void onOpened() {
                                             webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
                                         }

            @Override
                                         public void onError(int errorCode) {
                                             //This will be called if the camera could not be opened
                                         }
                                     }

        );

        Pose2d startPose = new Pose2d(16.62, -63.42, Math.toRadians(90.00));
        drive.setPoseEstimate(startPose);
        Trajectory right = drive.trajectoryBuilder(startPose)
                .splineTo(new Vector2d(22.69, -43.41), Math.toRadians(90.00))
                .build();

        Trajectory middle = drive.trajectoryBuilder(startPose)
                .splineTo(new Vector2d(13.22, -39.31), Math.toRadians(90.00))
                .build();

        Trajectory left = drive.trajectoryBuilder(startPose)
                .splineTo(new Vector2d(8.93, -39.13), Math.toRadians(140.00))
                .build();


        Trajectory backup_middle = drive.trajectoryBuilder(right.end())
                .back(20)
                .build();

        Trajectory backup_right = drive.trajectoryBuilder(right.end())
                .back(10)
                .build();
        Trajectory middle_park = drive.trajectoryBuilder(backup_middle.end())
                .strafeRight(25)
                .build();
        Trajectory right_park = drive.trajectoryBuilder(backup_right.end())
                .strafeRight(20)
                .build();
        Trajectory backup_left = drive.trajectoryBuilder(left.end())
                .back(30)
                .build();

        Trajectory left_strafe = drive.trajectoryBuilder(backup_left.end())
                .strafeRight(6)
                .build();
        Trajectory left_park = drive.trajectoryBuilder(left_strafe.end())
                .back(9)
                .build();

        waitForStart();
        if (isStopRequested()) return;


        switch (detector.getLocation()) {
            case LEFT:
                drive.followTrajectory(middle);
                drive.followTrajectory(backup_middle);
                //lift.moveToTarget(Lift.LiftPos.LOW);
                drive.followTrajectory(middle_park);
                break;
            case NOT_FOUND:
                drive.followTrajectory(left);
                drive.followTrajectory(backup_left);
                drive.followTrajectory(left_strafe);
                drive.followTrajectory(left_park);
                break;
            case RIGHT:
                drive.followTrajectory(right);
                drive.followTrajectory(backup_right);
                drive.followTrajectory(right_park);
                break;
        }






        webcam.stopStreaming();
    }
}



