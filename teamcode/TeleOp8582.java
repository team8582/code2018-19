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


        //constants (change as needed) --> boolean values for claw/grabber here
        boolean clawPrevState = false;    //claw servos previous state variable
        boolean clawCurrState = false;    //claw servos current state variable
        boolean clawServo = false;        //claw servos is in open position

        double OPEN = -1.0;     //left servo open position
        final double CLOSE = .9;   //left servo closed position


        waitForStart();
        runtime.reset();
        
        while (opModeIsActive()) {
            telemetry.update();

            // set wheel motors
            robot.leftDrive.setPower(gamepad1.left_stick_y);
            robot.rightDrive.setPower(gamepad1.right_stick_y);
            robot.frontDrive.setPower(gamepad1.right_stick_x);
            robot.backDrive.setPower(gamepad1.left_stick_x);

            //set lift motors
            robot.liftTop.setPower(gamepad2.right_stick_y);
            //robot.liftBottom.setPower(gamepad2.right_stick_y);

            //set current state of the claw to the state of the A button on gamepad2
            clawCurrState = !gamepad2.a;
            telemetry.addData("gamepad 2 a value:", gamepad2.a);

            //set servo
            if ((clawCurrState) && (clawCurrState != clawPrevState)) {   //button is transitioning to a pressed state
                clawServo = !clawServo;  //variable goes from false to true, so the servos should close
            }
            // update previous state variable.
            clawPrevState = clawCurrState;
            if (clawServo) {
                robot.liftServo.setPosition(CLOSE);   //set servos to closed position
            } else {
                robot.liftServo.setPosition(OPEN); //set servos to open position
            }
            
        }
        
        
    }
    
    
}
