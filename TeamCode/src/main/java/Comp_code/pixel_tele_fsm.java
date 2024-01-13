
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

import org.firstinspires.ftc.robotcore.external.Telemetry;
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


        // public static int HIGH = 500; //2900 = HIGH
//        public ElapsedTime runtime = new ElapsedTime();
        public PIDController controller;
        public static double p = 0.005, i = 0, d = 0.0; // d = dampener (dampens arm movement and is scary). ignore i
        public static double f = 0.0007;  // prevents arm from falling from gravity



        public enum LiftPos{
            START,
            LOW,
            MID,
            MIDHIGH,
            LOW_AUTO,
            HIGH,
            MANUAL
        }
        public DcMotorEx larm;
        public DcMotorEx rarm;
        public static int START_POS = 0;
        public static int LOW_POS = 1030;
        public static int MID_POS = 1460;
        public static int MID_HIGH_POS = 1750;
//max 2320
        public static int HIGH_POS = 2250;
        public static int LOW_AUTO = 900;
        public static int liftTarget = 0; // target position

//    int MANUAL = larm.getCurrentPosition() +20;

        private MultipleTelemetry tl;



//    int MANUAL = larm.getCurrentPosition() +20;

        //private MultipleTelemetry tl;
        double waitTime1 = .5;
        double waitTime2 = 0;
        double waitTime3 = 1.05;

        ElapsedTime waitTimer1 = new ElapsedTime();
        ElapsedTime waitTimer2 = new ElapsedTime();
        ElapsedTime waitTimer3 = new ElapsedTime();
        ElapsedTime runtime = new ElapsedTime();



        double SpeedAdjust = 1;

        double servospeed = 0.5;

        double lDropPos = 0;
        double rDropPos = 0;

        enum elbowDownState { //INTAKE
            START,
            INTAKE,
            WRIST

        }
        enum elbowUpState { //OUTTAKE
            START,
            OUTTAKE,


        }
        @Override
        public void runOpMode() throws InterruptedException {

            robot.init(hardwareMap);
            Lift lift = new Lift(hardwareMap, telemetry);

            robot.wrist.setPosition(0.999);

            robot.raxon.setPosition(.87);
            robot.laxon.setPosition(.12);
            robot.drone.setPosition(.5);
//            robot.ldrop.setPosition(0.214);
//            robot.rdrop.setPosition(0.175);

            waitForStart();

            if (isStopRequested()) return;

            elbowUpState outtake = elbowUpState.START;
            elbowDownState intake = elbowDownState.START;

            //LiftPos liftTarget = LiftPos.START;
            liftTarget = 0;
            while (opModeIsActive() && !isStopRequested()) {

                switch (outtake) {
                    case START:
                        if (gamepad2.dpad_down || gamepad2.dpad_left || gamepad2.dpad_right || gamepad2.dpad_up) {
                            //robot.grab.setPosition(CLOSE);
                            //robot.wrist.setPosition(.75);
                            waitTimer1.reset();
                            outtake = elbowUpState.OUTTAKE; //OUTTAKE POSITIONS, DIF HEIGHTS (black frame thing for pixels)
                        }
                        break;
                    case OUTTAKE:
                        if(waitTimer1.seconds() >= waitTime1) {
                            robot.wrist.setPosition(.001);
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
                            robot.raxon.setPosition(.79);
                            robot.laxon.setPosition(.21);
                            robot.wrist.setPosition(.8);

                            waitTimer3.reset();
                            intake = elbowDownState.WRIST;
                        }
                        break;
                    case WRIST:
                        if(waitTimer3.seconds() >= waitTime3) {

                            robot.wrist.setPosition(.999);

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
                    while(runtime.seconds() <= .36){
                        robot.wheel.setPosition(.32);
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

                }
                else if (gamepad1.circle) {
                    runtime.reset();
                    while(runtime.seconds() <= .7){
                        robot.wheel.setPosition(.3);
                    }
                    robot.wheel.setPosition(.5);
                    runtime.reset();
                }
                //lift
                if (gamepad2.dpad_down) {

                    liftTarget = LOW_POS;
                }

                else if (gamepad2.dpad_left) {
                    liftTarget = MID_POS;

                }
                else if (gamepad2.dpad_right) {
                    liftTarget = MID_HIGH_POS;

                }
                else if (gamepad2.dpad_up) {
                    liftTarget = HIGH_POS;
                }
                if (gamepad2.cross) {
                    liftTarget = START_POS;
                }
//                else if (gamepad2.circle) {
//                    liftTarget = Lift.LiftPos.MANUAL;
//                }

//                if (gamepad1.dpad_down) {//intake
//                    robot.ldrop.setPosition(.179);
//                    robot.rdrop.setPosition(.219);
//                }
//                if (gamepad1.dpad_up) { //away
//
//                    //lDropPos = robot.ldrop.getPosition() -.05;
//                   // rDropPos = robot.ldrop.getPosition() +.05;
//                    robot.ldrop.setPosition(0.2115);
//                    robot.rdrop.setPosition(.176);
//
//                }
//                if (gamepad1.dpad_left) {//stack
//                   // double change = 0;
////                    lDropPos = robot.ldrop.getPosition() -.01;
////                     rDropPos = robot.ldrop.getPosition() +.01;
//                    robot.ldrop.setPosition(.19);
//                    robot.rdrop.setPosition(.195);
//
//                }
//                if (gamepad1.dpad_right) {//stack
//                    robot.ldrop.setPosition(.185);
//                    robot.rdrop.setPosition(.20);
//                }
////
//
//                    //elbow
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

//                    if (gamepad2.triangle) {
//                        robot.drone.setPosition(.8);
//                    }


                    if(gamepad2.left_bumper && gamepad2.right_bumper) {
                        robot.drone.setPosition(.8);
                    }

                    else if (gamepad2.circle) {
                        liftTarget = larm.getCurrentPosition() + 20;
                    }
                    else if (gamepad2.square) {
                        liftTarget = larm.getCurrentPosition() - 20;
                    }
                int drone_target = 0;

                    if (gamepad2.triangle) {
                        drone_target = 20;
                        robot.climb.setPower(1);
                        robot.climb.setTargetPosition(drone_target);
                    }
//                     if(!gamepad2.circle){
//                         lift.larm.setPower(0.0007);
//                         lift.rarm.setPower(-0.0007);
//                     }
//                    } else if (gamepad2.circle){
                     //   double lpwr = .5;
//                        lift.larm.setPower(lpwr);
                          //lift.larm.setPower(0);
//                    }




                lift.update();
                //robot.climb.setPower(1);
                robot.climb.setTargetPosition(drone_target);

                telemetry.update();
                robot.wheel.setPosition(servospeed);


            }
        }
        //}

        //}
        class Lift {
            public Lift(HardwareMap hardwareMap, Telemetry telemetry) {
                // Beep boop this is the the constructor for the lift
                // Assume this sets up the lift hardware

                controller = new PIDController(p, i, d);
                tl = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

                larm = hardwareMap.get(DcMotorEx.class, "Llift");
                rarm = hardwareMap.get(DcMotorEx.class, "Rlift");


                larm.setDirection(DcMotorEx.Direction.FORWARD);
                rarm.setDirection(DcMotorEx.Direction.REVERSE);

                larm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rarm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                larm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                rarm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }

            public void update() {
//                setTarget(target);
                // Beep boop this is the lift update function
                // Assume this runs some PID controller for the lift


                controller.setPID(p, i, d);

                int larmPos = larm.getCurrentPosition();
                int rarmPos = rarm.getCurrentPosition();

                double Lpid = controller.calculate(larmPos, liftTarget);
                double Rpid = controller.calculate(rarmPos, liftTarget);

                // double Lff = Math.cos(Math.toRadians(LiftTarget / ticks_in_degree)) * f; //* (12/voltageSensor.getVoltage()
                // double Rff = Math.cos(Math.toRadians(LiftTarget / ticks_in_degree)) * f; // * (12/voltageSensor.getVoltage()

                double Lpower = Lpid + f;
                double Rpower = Rpid + f;

                larm.setPower(Lpower);
                rarm.setPower(Rpower);


                tl.update();
            }

            public boolean atTarget() {
                return controller.atSetPoint();
            }

//            public void moveToTarget(LiftPos target) {
//
//                setTarget(target);
//
//                while (!atTarget()) {
//                    update(target);
//
//                }
//            }

//            public void setTarget(LiftPos target) {
//                int encoderTarget = 0;
//                // Beep boop this is the lift update function
//                // Assume this runs some PID controller for the lift
//                switch (target) {
//                    //int MANUAL = larm.getCurrentPosition() +20;
//
//                    case START:
//                        encoderTarget = START_POS;
//                        break;
//                    case LOW:
//                        encoderTarget = LOW_POS;
//                        break;
//
//                    case MID:
//                        encoderTarget = MID_POS;
//                        break;
//                    case MIDHIGH:
//                        encoderTarget = MID_HIGH_POS;
//                        break;
//                    case HIGH:
//                        encoderTarget = HIGH_POS;
//                        break;
//
//                    case LOW_AUTO:
//                        encoderTarget = LOW_AUTO;
//                        break;
//                    case MANUAL:
//                        encoderTarget = larm.getCurrentPosition() + 20;
//                        break;
//                }

//                controller.setSetPoint(encoderTarget);
//            }

        }
    }

