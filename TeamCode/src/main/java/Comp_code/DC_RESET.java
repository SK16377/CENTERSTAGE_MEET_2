/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package Comp_code;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

//start position thingy Ltarget = 440 , Rtarget = 439
@Disabled
@TeleOp(name="VOLTAGE BACKUP")

public class DC_RESET extends LinearOpMode {
    bot_map robot = new bot_map();
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    ;

    double SpeedAdjust = 1;


    private DcMotor larm;
    private DcMotor rarm;



    @Override
    public void runOpMode() {

        robot.init(hardwareMap);
        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        robot.wrist.setPosition(0.999);

        robot.raxon.setPosition(.87);
        robot.laxon.setPosition(.12);

        larm = hardwareMap.get(DcMotor.class,"Llift");
        rarm = hardwareMap.get(DcMotor.class,"Rlift");
        // Initialize the hardware variables. Note that the strings used here as parameters

        rarm.setDirection(DcMotor.Direction.REVERSE);
        larm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rarm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        larm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rarm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //robot.leftClaw.setPosition(L_OPEN);
        larm.setPower(0);
        rarm.setPower(0);



        telemetry.addData("Status", "Initialized");
        telemetry.update();



        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();
        //   START_POS();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {


            if (gamepad1.left_trigger ==1 ){
                larm.setPower(-.8);
                larm.setPower(-.8);
            } else {
                larm.setPower(0);
                larm.setPower(0);
            }


            telemetry.addData("Status", "Run Time: " + runtime.toString());
            //telemetry.addData("Endgame:", endgame);
            telemetry.update();
        }
    }
}


