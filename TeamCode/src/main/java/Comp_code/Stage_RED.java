package Comp_code;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import Comp_code.Lift;
import Comp_code.bot_map;
import opencv.CenterstageDetector;
import opencv.redAudiencePipeline;

//import com.acmerobotics.roadrunner.geometry.Pose2d;


//import com.acmerobotics.roadrunner.trajectoryBuilder;


@Autonomous(name="Red_STACK", group="Auto")
public class Stage_RED extends LinearOpMode {
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

        Trajectory left_stage = drive.trajectoryBuilder(strafe_left.end())
                .splineTo(new Vector2d(-24.76, -17.86), Math.toRadians(0.00))
                .build();
        Trajectory left_straight = drive.trajectoryBuilder(left_stage.end())
                .forward(47)
                .build();

        Trajectory left_drop = drive.trajectoryBuilder(left_straight.end())
                .splineTo(new Vector2d(41.24, -33.9), Math.toRadians(0.00))
                .build();
        Trajectory deposit_left = drive.trajectoryBuilder(left_drop.end())
                .forward(7.4)
                .build();
        Trajectory away_left = drive.trajectoryBuilder(deposit_left.end())
                .back(4.5)
                .build();
        Trajectory left_park = drive.trajectoryBuilder(away_left.end())
                .strafeLeft(15)
                .build();

        Trajectory backup_right = drive.trajectoryBuilder(left.end())
                .back(6)
                .build();


        Trajectory right_stage = drive.trajectoryBuilder(backup_right.end())
                .splineTo(new Vector2d(-24.76, -17.86), Math.toRadians(0.00))
                .build();
        Trajectory right_straight = drive.trajectoryBuilder(right_stage.end())
                .forward(47)
                .build();
        Trajectory right_drop = drive.trajectoryBuilder(right_straight.end())
                .splineTo(new Vector2d(37.75, -47), Math.toRadians(0.00))
                .build();
        Trajectory deposit_right = drive.trajectoryBuilder(right_drop.end())
                .forward(6.75)
                .build();
        Trajectory away_right = drive.trajectoryBuilder(deposit_right.end())
                .back(5.8)
                .build();
        Trajectory after_right = drive.trajectoryBuilder(away_right.end())
                .strafeLeft(25)
                .build();
        Trajectory right_park = drive.trajectoryBuilder(after_right.end())
                .strafeLeft(16)
                .build();
        Trajectory backup_middle = drive.trajectoryBuilder(middle.end())
                .back(2)
                .build();
        Trajectory strafe_middle = drive.trajectoryBuilder(backup_middle.end())
                .strafeLeft(16)
                .build();
        Trajectory stage_middle = drive.trajectoryBuilder(strafe_middle.end())
                .splineTo(new Vector2d(-42.88, -10.68), Math.toRadians(0.00))
                .build();
        Trajectory straight_middle = drive.trajectoryBuilder(stage_middle.end())
                .forward(57)
                .build();
        Trajectory drop_middle = drive.trajectoryBuilder(straight_middle.end())
                .splineTo(new Vector2d(39.17, -34), Math.toRadians(0.00))
                .build();
        Trajectory deposit_middle = drive.trajectoryBuilder(drop_middle.end())
                .forward(6)
                .build();
        Trajectory away_middle = drive.trajectoryBuilder(deposit_middle.end())
                .back(5.5)
                .build();
        Trajectory middle_after = drive.trajectoryBuilder(away_middle.end())
                .strafeLeft(25)
                .build();
        Trajectory middle_park = drive.trajectoryBuilder(middle_after.end())
                .forward(7.7)
                .build();
        waitForStart();
        if (isStopRequested()) return;
       redAudiencePipeline.Location location = detector.getLocation();
        switch (location) {
            case LEFT: //left
                drive.followTrajectory(left);
                drive.followTrajectory(backup_left);
                drive.followTrajectory(strafe_left);
                drive.followTrajectory(left_stage);
                drive.followTrajectory(left_straight);
                drive.followTrajectory(left_drop);
                scoreLow(deposit_left, away_left);
                drive.followTrajectory(left_park);


                break;
            case NOT_FOUND: //right
                drive.followTrajectory(right);
                drive.followTrajectory(backup_right);

                drive.followTrajectory(right_stage);
                drive.followTrajectory(right_straight);
                drive.followTrajectory(right_drop);
                scoreLow(deposit_right, away_right);
                drive.followTrajectory(after_right);
               // drive.followTrajectory(right_park);
                break;
            case RIGHT: //middle
                drive.followTrajectory(middle);
                drive.followTrajectory(backup_middle);
                drive.followTrajectory(strafe_middle);
                drive.followTrajectory(stage_middle);
                drive.followTrajectory(straight_middle);
                drive.followTrajectory(drop_middle);
                scoreLow(deposit_middle, away_middle);
                drive.followTrajectory(middle_after);
               // drive.followTrajectory(middle_park);
                break;

        }




        webcam.stopStreaming();
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
}



