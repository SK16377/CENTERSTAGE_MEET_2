
package Comp_code;/*
     * Some declarations that are boilerplate are
     * skipped for the sake of brevity.
     * Since there are no real values to use, named constants will be used.
     */

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
//import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
//import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDController;
//import Comp_code.bot_map;

//import Testing.teleptest;


    @TeleOp(name="pixel_tele_fsm")
    public class pixel_tele_fsm extends LinearOpMode {


        // Some hardware access boilerplate; these would be initialized in init()
        // the lift motor, it's in RUN_TO_POSITION mode

        //TELEmap robot   = new TELEmap();
        bot_map robot = new bot_map();


        // used with the dump servo, this will get covered in a bit
        ElapsedTime toggleTimer = new ElapsedTime();
        //double toggleTime = .25;

        public PIDController controller;
        public static double p = 0.005, i = 0, d = 0.0; // d = dampener (dampens arm movement and is scary). ignore i
        public static double f = .002;  // prevents arm from falling from gravity


        public static int LiftTarget = 0; // target position

        public static int START_POS = 0;
        public static int LOW = 2100; //1208 = LOW
        public static int MID = 2530; //2078 = MID
        // public static int HIGH = 500; //2900 = HIGH
//        public ElapsedTime runtime = new ElapsedTime();
        double waitTime1 = .5;
        double waitTime2 = .5;
        double waitTime3 = .5;


        ElapsedTime waitTimer1 = new ElapsedTime();
        ElapsedTime waitTimer2 = new ElapsedTime();
        ElapsedTime waitTimer3 = new ElapsedTime();
        ElapsedTime runtime = new ElapsedTime();

        public DcMotorEx larm;
        public DcMotorEx rarm;

        double SpeedAdjust = 1;

        double servospeed = 0.5;

        enum elbowDownState { //INTAKE
            START,
            INTAKE,


        }
        enum elbowUpState { //OUTTAKE
            START,
            OUTTAKE,


        }
        @Override
        public void runOpMode() throws InterruptedException {

            robot.init(hardwareMap);
            Lift lift = new Lift(hardwareMap);

            robot.wrist.setPosition(0.99);
            robot.raxon.setPosition(.89);
            robot.laxon.setPosition(.11);


            waitForStart();

            if (isStopRequested()) return;

            elbowUpState outtake = elbowUpState.START;
            elbowDownState intake = elbowDownState.START;

            LiftTarget = 0;

            while (opModeIsActive() && !isStopRequested()) {

                switch (outtake) {
                    case START:
                        if (gamepad2.dpad_down || gamepad2.dpad_left || gamepad2.dpad_right) {
                            //robot.grab.setPosition(CLOSE);
                            //robot.wrist.setPosition(.75);
                            waitTimer1.reset();
                            outtake = elbowUpState.OUTTAKE; //OUTTAKE POSITIONS, DIF HEIGHTS (black frame thing for pixels)
                        }
                        break;
                    case OUTTAKE:
                        if(waitTimer1.seconds() >= waitTime1) {
                            robot.wrist.setPosition(.01);
                            robot.raxon.setPosition(.11);
                            robot.laxon.setPosition(.89);
                            waitTimer1.reset();
                            outtake = elbowUpState.START;
                        }
                        break;

                }

                switch (intake) {
                    case START:
                        if (gamepad2.cross) {

                            waitTimer2.reset();
                            intake = elbowDownState.INTAKE; //OUTTAKE POSITIONS, DIF HEIGHTS (black frame thing for pixels)
                        }
                        break;
                    case INTAKE:
                        if(waitTimer2.seconds() >= waitTime2) {
                            robot.wrist.setPosition(.95);
                            robot.raxon.setPosition(.88);
                            robot.laxon.setPosition(.12);

                            intake = elbowDownState.START;
                        }
                        break;

                }

                 //intake buttons 1x = board tri = expel
                if (gamepad1.right_trigger > 0) { //out
                    robot.intake.setPower(-4);

                }else if (gamepad1.left_trigger > 0) { //in
                    //servospeed = 0.5;
                    robot.wheel.setPosition(.9);
                    robot.intake.setPower(2);
                    //robot.wheel.setPower(2); THE WHEEL IS A SERVO OOPS
                } else {
                    robot.intake.setPower(0);
                    //robot.wheel.setPower(0);
                    robot.wheel.setPosition(.5);
                }
                if(gamepad1.cross){
                    runtime.reset();
                    while(runtime.seconds() <= .5){
                        robot.wheel.setPosition(.1);
                    }
                    robot.wheel.setPosition(.5);
                    runtime.reset();
                }

                //drive
                robot.leftFront.setPower((gamepad1.left_stick_y - gamepad1.left_stick_x + gamepad1.right_stick_x) / SpeedAdjust);
                robot.rightFront.setPower((gamepad1.left_stick_y + gamepad1.left_stick_x - gamepad1.right_stick_x) / SpeedAdjust);
                robot.leftBack.setPower((gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x) / SpeedAdjust);
                robot.rightBack.setPower((gamepad1.left_stick_y - gamepad1.left_stick_x - gamepad1.right_stick_x) / SpeedAdjust);

//                robot.rightBack.setPower((-gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x) / SpeedAdjust);
//                robot.leftBack.setPower((-gamepad1.left_stick_y - gamepad1.left_stick_x - gamepad1.right_stick_x) / SpeedAdjust);
//                robot.rightFront.setPower((-gamepad1.left_stick_y - gamepad1.left_stick_x + gamepad1.right_stick_x) / SpeedAdjust);
//                robot.leftFront.setPower((-gamepad1.left_stick_y + gamepad1.left_stick_x - gamepad1.right_stick_x) / SpeedAdjust);
                //climb
                if (gamepad2.right_trigger > 0) {
                    robot.climb.setPower(2);

                }else if (gamepad2.left_trigger > 0) {
                    robot.climb.setPower(-2);
                }else {
                    robot.climb.setPower(0);
                }

                if (gamepad1.square) {
                    robot.wrist.setPosition(.99);
                }
                else if (gamepad1.circle) {
                    robot.wrist.setPosition(.3);
                }
                //lift
                if (gamepad2.dpad_down) {
                    //lift_speed = 0.2;
                    // lift_speed = 1;
                    LiftTarget = LOW;
                }
                else if (gamepad2.dpad_right) {
                    LiftTarget = MID;
                }
                else if (gamepad2.dpad_left) {
                    LiftTarget = MID;
                }
                if (gamepad2.cross) {
                    LiftTarget = 0;
                }



                    //elbow
//                if(gamepad2.circle){ //outake pos
//                    robot.raxon.setPosition(.1);
//                    robot.laxon.setPosition(.9);
//                    //robot.wrist.setPosition(.3);
//                }
//                if(gamepad2.square){ //intake pos
//                    robot.raxon.setPosition(.9);
//                    robot.laxon.setPosition(.1);
//                    // robot.wrist.setPosition(.3);
//                }

                //wrist
                // if(gamepad2.triangle){ // outake pos (not fixed)
                //     robot.wrist.setPosition(.75);
                // }
                // if(gamepad2.cross){ //intake pos, comented out bc i used the same thing elsewhere
                //     robot.wrist.setPosition(.15);
                // }

                //drive speed adjust
                if (gamepad1.left_bumper) {
                    SpeedAdjust = 4;
                } else if (gamepad1.right_bumper) {
                    SpeedAdjust = 1;
                }

                //drone

                    if (gamepad2.circle) {
                        robot.drone.setPosition(.3);
                    }


                lift.update();
                telemetry.update();
                robot.wheel.setPosition(servospeed);
            }
        }
        //}

        //}

        class Lift {
            public Lift(HardwareMap hardwareMap) {
                // Beep boop this is the the constructor for the lift
                // Assume this sets up the lift hardware

                controller = new PIDController(p, i, d);
                telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

                larm = hardwareMap.get(DcMotorEx.class,"Llift");
                rarm = hardwareMap.get(DcMotorEx.class,"Rlift");


                larm.setDirection(DcMotorEx.Direction.FORWARD);
                rarm.setDirection(DcMotorEx.Direction.REVERSE);

                larm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rarm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                larm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                rarm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }

            public void update() {
                // Beep boop this is the lift update function
                // Assume this runs some PID controller for the lift

                controller.setPID(p, i, d);

                int larmPos = larm.getCurrentPosition();
                int rarmPos = rarm.getCurrentPosition();

                double Lpid = controller.calculate(larmPos, LiftTarget);
                double Rpid = controller.calculate(rarmPos, LiftTarget);

                // double Lff = Math.cos(Math.toRadians(LiftTarget / ticks_in_degree)) * f; //* (12/voltageSensor.getVoltage()
                // double Rff = Math.cos(Math.toRadians(LiftTarget / ticks_in_degree)) * f; // * (12/voltageSensor.getVoltage()

                double Lpower = Lpid + f;
                double Rpower = Rpid + f;

                larm.setPower(Lpower);
                rarm.setPower(Rpower);

//            telemetry.addData("Lpos", larmPos);
//            telemetry.addData("Rpos", rarmPos);
//            telemetry.addData("Ltarget", LiftTarget);
//            telemetry.addData("Rtarget", LiftTarget);
                // telemetry.addData("cone", cone);
                //telemetry.addData("SSVar", SSVar);
                telemetry.update();
            }
        }

    }
