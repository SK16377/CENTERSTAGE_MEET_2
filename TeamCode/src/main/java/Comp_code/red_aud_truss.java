package Comp_code;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import opencv.redAudiencePipeline;

//import com.acmerobotics.roadrunner.geometry.Pose2d;


//import com.acmerobotics.roadrunner.trajectoryBuilder;


@Autonomous(name="Red_aud_truss", group="Auto")
public class red_aud_truss extends LinearOpMode {
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
        redAudiencePipeline detector = new redAudiencePipeline(telemetry);
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

        Pose2d startPose = new Pose2d(-40, -63.42, Math.toRadians(90));
        drive.setPoseEstimate(startPose);
        Trajectory left = drive.trajectoryBuilder(startPose)
                .splineTo(new Vector2d(-46.35, -43.81), Math.toRadians(90))
                .build();

        Trajectory middle = drive.trajectoryBuilder(startPose)
                .splineTo(new Vector2d(-37.5, -38.6), Math.toRadians(90))
                .build();


        Trajectory right = drive.trajectoryBuilder(startPose)
                .splineTo(new Vector2d(-33, -39), Math.toRadians(40.00))
                .build();



        Trajectory backup_left = drive.trajectoryBuilder(left.end())
                .back(3)
                .build();
        Trajectory strafe_left = drive.trajectoryBuilder(backup_left.end())
                .strafeRight(13)
                .build();
        Trajectory left_truss = drive.trajectoryBuilder(backup_left.end())
                .lineToLinearHeading(new Pose2d(-42.5, -60, Math.toRadians(0.00)))
                .build();
        Trajectory left_strafe_ = drive.trajectoryBuilder(left_truss.end())
                .strafeRight(8.85)
                .build();
        Trajectory left_undertruss = drive.trajectoryBuilder(left_truss.end())
                .forward(55)
                .build();
        Trajectory leftat_back = drive.trajectoryBuilder(left_undertruss.end())
                .lineToLinearHeading(new Pose2d(44, -29.5, Math.toRadians(0.00)))
                .build();


        Trajectory deposit_left = drive.trajectoryBuilder(leftat_back.end())
                .forward(4.479)
                .build();
        Trajectory away_left = drive.trajectoryBuilder(deposit_left.end())
                .back(4.5)
                .build();
        Trajectory left_park = drive.trajectoryBuilder(away_left.end())
                .strafeRight(32.5)
                .build();
        Trajectory backup_right = drive.trajectoryBuilder(right.end())
                .back(4)
                .build();
        Trajectory right_truss = drive.trajectoryBuilder(backup_right.end())
                .lineToLinearHeading(new Pose2d(-42.5, -64.5, Math.toRadians(0.00)))
                .build();

        Trajectory right_undertruss = drive.trajectoryBuilder(right_truss.end())
                .forward(55)
                .build();
        Trajectory rightat_back = drive.trajectoryBuilder(right_undertruss.end())
                .lineToLinearHeading(new Pose2d(39.25, -49.15, Math.toRadians(0.00)))
                .build();
//44 middle

        Trajectory right_deposit = drive.trajectoryBuilder(rightat_back.end())
                .forward(4.6)
                .build();
        Trajectory away_right = drive.trajectoryBuilder(right_deposit.end())
                .back(4.5)
                .build();
        Trajectory right_park = drive.trajectoryBuilder(away_right.end())
                .strafeRight(19.3)
                .build();



        Trajectory backup_middle = drive.trajectoryBuilder(middle.end())
                .back(4)
                .build();
        Trajectory middle_truss = drive.trajectoryBuilder(backup_middle.end())
                .lineToLinearHeading(new Pose2d(-42.5, -61.85, Math.toRadians(0.00)))
                .build();

        Trajectory middle_undertruss = drive.trajectoryBuilder(middle_truss.end())
                .forward(55)
                .build();
        Trajectory middleat_back = drive.trajectoryBuilder(middle_undertruss.end())
                .lineToLinearHeading(new Pose2d(39.25, -39.9, Math.toRadians(0.00)))
                .build();
//44 middle

        Trajectory middle_deposit = drive.trajectoryBuilder(middleat_back.end())
                .forward(5.99)
                .build();
        Trajectory middle_right = drive.trajectoryBuilder(middle_deposit.end())
                .back(4.5)
                .build();
        Trajectory away_middle = drive.trajectoryBuilder(middle_deposit.end())
                .back(4.5)
                .build();
        Trajectory middle_park = drive.trajectoryBuilder(away_middle.end())
                .strafeRight(25)
                .build();
        waitForStart();
        if (isStopRequested()) return;
        redAudiencePipeline.Location location = detector.getLocation();
        switch (location) {
            case LEFT: //left
                drive.followTrajectory(left);
                drive.followTrajectory(backup_left);
                drive.followTrajectory(left_truss);
//                drive.followTrajectory(left_strafe_);
                drive.followTrajectory(left_undertruss);
                drive.followTrajectory(leftat_back);
                scoreLow(deposit_left, away_left, left_park);



//                drive.followTrajectory(left_stage);
//                drive.followTrajectory(left_straight);
//                drive.followTrajectory(left_drop);
//                scoreLow(deposit_left, away_left);
//                drive.followTrajectory(left_park);


                break;
            case NOT_FOUND: //right
                drive.followTrajectory(right);
                drive.followTrajectory(backup_right);
                drive.followTrajectory(right_truss);
//                drive.followTrajectory(left_strafe_);
                drive.followTrajectory(right_undertruss);
                drive.followTrajectory(rightat_back);
                scoreLow(right_deposit, away_right, right_park);

//                drive.followTrajectory(right_stage);
//                drive.followTrajectory(right_straight);
//                drive.followTrajectory(right_drop);
//                scoreLow(deposit_right, away_right);
//                drive.followTrajectory(after_right);
                // drive.followTrajectory(right_park);
                break;
            case RIGHT: //middle
                drive.followTrajectory(middle);
                drive.followTrajectory(backup_middle);
                drive.followTrajectory(middle_truss);
//                drive.followTrajectory(left_strafe_);
                drive.followTrajectory(middle_undertruss);
                drive.followTrajectory(middleat_back);
                scoreLow(middle_deposit, away_middle, middle_park);
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

        drive.followTrajectory(park);
        arm.autonparkpos();
        lift.moveToTarget(Lift.LiftPos.START);
        arm.intakePos();


    }
}



