package Comp_code;


import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Arm {
    public Servo wheel;
    public Servo raxon;
    public Servo wrist;
    public Servo laxon;
    Telemetry telemetry;


    public Arm(bot_map robot, Telemetry telemetry){
        this.wheel = robot.wheel;
        this.raxon = robot.raxon;
        this.wrist = robot.wrist;
        this.laxon = robot.laxon;
        this.telemetry = telemetry;
    }
    public Arm(HardwareMap hwMap, Telemetry telemetry){
        wheel = hwMap.get(Servo.class, "wheel");
        raxon = hwMap.get(Servo.class, "raxon");
        laxon = hwMap.get(Servo.class, "laxon");
        wrist = hwMap.get(Servo.class, "wrist");
        this.telemetry = telemetry;
    }

    public void goToScoringPos(){
       wrist.setPosition(.001);
       raxon.setPosition(.11);
       laxon.setPosition(.89);
    }

    public void deposit(double sec){
        ElapsedTime runtime = new ElapsedTime();
        runtime.reset();
        while(runtime.seconds() <= sec){
            wheel.setPosition(.25);
        }
        wheel.setPosition(.5);
        runtime.reset();
    }

    public void intakePos(){
        wrist.setPosition(.999);

    }
    public void autonparkpos(){
        raxon.setPosition(.79);
        laxon.setPosition(.21);
        wrist.setPosition(.8);
    }
    public void wheelIntake(double sec){
        ElapsedTime runtime = new ElapsedTime();
        runtime.reset();
        while(runtime.seconds() <= sec){
            wheel.setPosition(.99);
        }
        wheel.setPosition(.5);
        runtime.reset();
    }
}
