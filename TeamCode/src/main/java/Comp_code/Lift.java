package Comp_code;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;


public class Lift {
    public PIDController controller;
    public static double p = 0.005, i = 0, d = 0.0; // d = dampener (dampens arm movement and is scary). ignore i
    public static double f = .002;  // prevents arm from falling from gravity



    public enum LiftPos{
        START,
        LOW,
        MID, LOW_AUTO
    }
    public static int START_POS = 0;
    public static int LOW_POS = 2100; //1208 = LOW
    public static int MID_POS = 2530; //2078 = MID
    public static int LOW_AUTO = 1650; //2078 = MID

    private MultipleTelemetry tl;
    public DcMotorEx larm;
    public DcMotorEx rarm;

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

    public void update(LiftPos target) {
      setTarget(target);
        // Beep boop this is the lift update function
        // Assume this runs some PID controller for the lift


        controller.setPID(p, i, d);

        int larmPos = larm.getCurrentPosition();
        int rarmPos = rarm.getCurrentPosition();

        double Lpid = controller.calculate(larmPos);
        double Rpid = controller.calculate(rarmPos);

        // double Lff = Math.cos(Math.toRadians(LiftTarget / ticks_in_degree)) * f; //* (12/voltageSensor.getVoltage()
        // double Rff = Math.cos(Math.toRadians(LiftTarget / ticks_in_degree)) * f; // * (12/voltageSensor.getVoltage()

        double Lpower = Lpid + f;
        double Rpower = Rpid + f;

        larm.setPower(Lpower);
        rarm.setPower(Rpower);


        tl.update();
    }

    public boolean atTarget(){
        return controller.atSetPoint();
    }

    public void moveToTarget(LiftPos target){

        setTarget(target);

        while(!atTarget()){
            update(target);

        }
    }

    public void setTarget(LiftPos target){
        int encoderTarget = 0;
        // Beep boop this is the lift update function
        // Assume this runs some PID controller for the lift
        switch (target){

            case START:
                encoderTarget = START_POS;
                break;
            case LOW:
                encoderTarget = LOW_POS;
                break;

            case MID:
                encoderTarget = MID_POS;
                break;

            case LOW_AUTO:
                encoderTarget = LOW_AUTO;
                break;
        }

        controller.setSetPoint(encoderTarget);
    }
}

