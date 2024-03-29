package Comp_code;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import Comp_code.Lift;
import Comp_code.bot_map;
import opencv.detector_2_ranges;

//import com.acmerobotics.roadrunner.geometry.Pose2d;


//import com.acmerobotics.roadrunner.trajectoryBuilder;


@Autonomous(name="Red_BACK", group="Auto")
public class Drop_RED extends LinearOpMode {
    SampleMecanumDrive drive;
    OpenCvCamera webcam;
    Lift lift;
    Arm arm;
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
                .splineTo(new Vector2d(13.3, -39.5), Math.toRadians(90.00))
                .build();

        Trajectory left = drive.trajectoryBuilder(startPose)
                .splineTo(new Vector2d(8.93, -39.13), Math.toRadians(140.00))
                .build();


        Trajectory backup_middle = drive.trajectoryBuilder(middle.end())
                .back(3)
                .build();
        Trajectory drop_middle = drive.trajectoryBuilder(backup_middle.end())
                .lineToLinearHeading(new Pose2d(44.24, -39.75, Math.toRadians(0.00)))
                .build();
        Trajectory deposit_middle = drive.trajectoryBuilder(drop_middle.end())
                .forward(6.7)
                .build();
        Trajectory away_middle = drive.trajectoryBuilder(deposit_middle.end())
                .back(5)
                .build();
        Trajectory middle_park = drive.trajectoryBuilder(away_middle.end())
                .strafeRight(24)
                .build();
        Trajectory backup_right = drive.trajectoryBuilder(right.end())
                .back(3.6)
                .build();

        Trajectory right_drop = drive.trajectoryBuilder(backup_right.end())
                .lineToLinearHeading(new Pose2d(44.24, -45, Math.toRadians(0.00)))
                .build();
        Trajectory deposit_right = drive.trajectoryBuilder(right_drop.end())
                .forward(6)
                .build();
        Trajectory away_right = drive.trajectoryBuilder(deposit_right.end())
                .back(5)
                .build();
        Trajectory right_park = drive.trajectoryBuilder(away_right.end())
                .strafeRight(15)
                .build();

        Trajectory backup_left = drive.trajectoryBuilder(left.end())
                .back(5)
                .build();
        Trajectory left_drop = drive.trajectoryBuilder(backup_left.end())
                .lineToLinearHeading(new Pose2d(44.24, -31.2, Math.toRadians(0.00)))
                .build();

        Trajectory deposit_left = drive.trajectoryBuilder(left_drop.end())
                .forward(6.8)
                .build();
        Trajectory away_left = drive.trajectoryBuilder(deposit_left.end())
                .back(5)
                .build();
        Trajectory left_park = drive.trajectoryBuilder(away_left.end())
                .strafeRight(30)
                .build();

        waitForStart();
        if (isStopRequested()) return;
          detector_2_ranges.Location location = detector.getLocation();
        switch (location) {
            case LEFT: //middle
                drive.followTrajectory(middle);
                drive.followTrajectory(backup_middle);
                drive.followTrajectory(drop_middle);
                scoreLow(deposit_middle, away_middle, middle_park);

                break;
            case NOT_FOUND: //left
                drive.followTrajectory(left);
                drive.followTrajectory(backup_left);
                drive.followTrajectory(left_drop);
                scoreLow(deposit_left, away_left, left_park);

                break;
            case RIGHT: //right
                drive.followTrajectory(right);
                drive.followTrajectory(backup_right);
                drive.followTrajectory(right_drop);
                scoreLow(deposit_right, away_right, right_park);

                break;
        }




        webcam.stopStreaming();
    }
    public void scoreLow(Trajectory backdrop, Trajectory away, Trajectory park){



        arm.goToScoringPos();
        lift.moveToTarget(Lift.LiftPos.LOW_AUTO);

        drive.followTrajectory(backdrop);
        arm.deposit(1);
        drive.followTrajectory(away);
        arm.autonparkpos();
        drive.followTrajectory(park);

        lift.moveToTarget(Lift.LiftPos.START);
        arm.intakePos();


    }
}



