package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.opmode.TelemetryImpl;




@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="TeleOp8582")
//@Disabled
public class TeleOp8582 extends LinearOpMode {
    Hardware8582 robot = new Hardware8582();
    private ElapsedTime runtime = new ElapsedTime();


    @Override
    public void runOpMode() {

        robot.init(hardwareMap);
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        //constants (change as needed)

        boolean clawPrevState = false;    //claw servos previous state variable
        boolean clawCurrState = false;    //claw servos current state variable
        boolean clawServo = false;        //claw servos is in open position

        //boolean openPrevState = false;
        //boolean openCurrState = false;
        //boolean openPosition = false;

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
           telemetry.update();

           robot.leftDrive.setPower(gamepad1.left_stick_y);
           robot.rightDrive.setPower(gamepad1.right_stick_y);
           robot.frontDrive.setPower(gamepad1.right_stick_x);
           robot.backDrive.setPower(gamepad1.left_stick_x);
           robot.lift.setPower(gamepad2.right_stick_y);


            /*if ((clawCurrState) && (clawCurrState != clawPrevState)) {   //button is transitioning to a pressed state
                clawServo = !clawServo;  //variable goes from false to true, so the servos should close
            }
            // update previous state variable.
            clawPrevState = clawCurrState;
            if (clawServo) {
                    robot.corral.setPower(-.5);
                    sleep(300);
                    robot.corral.setPower(0);
            }
            else {

                    robot.corral.setPower(.5);
                    sleep(300);
                    robot.corral.setPower(0);
                //set servos to open position

            }
*/

           /* if ((openCurrState) && (openCurrState != openPrevState)) {
                openPosition = !openPosition;
            }
            openPrevState = openCurrState;
            if (openPosition) {
                OPEN_RIGHT = 0.2;
            } else {
                OPEN_RIGHT = 1.0;
            }
            */


        }


    }




}
