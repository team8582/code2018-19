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

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="autoColorTest")
public class autoColorTest extends LinearOpMode {
    Hardware8582 robot = new Hardware8582();
    private ElapsedTime runtime = new ElapsedTime();       //Create variable to keep track of elapsed time.
    static final double DRIVE_SPEED = 0.7;
    static final double TURN_SPEED = 0.5;
    final double COUNTS_PER_MOTOR_REV = 1120;     //Andymark Motor Encoder
    final double DRIVE_GEAR_REDUCTION = 1.0;      //This is < 1.0 if geared up
    final double WHEEL_DIAMETER_INCHES = 3.75;     //For figuring circumference
    final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) *
            (WHEEL_DIAMETER_INCHES * 3.1415);

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);

        //robot.colorSensor.enableLed(true); //turn on LED which causes jewel to reflect wavelengths for the sensor to read

        //turn on encoders
        robot.leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //robot.lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        idle();

        robot.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.frontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.backDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //robot.lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        boolean yellowFound = false;

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        /*//variables for different color reading
        int redValue = robot.colorSensor.red();
        int greenValue = robot.colorSensor.green();
        int blueValue = robot.colorSensor.blue();
        boolean isYellow = false;   //jewel is yellow
        if (((redValue > blueValue * 2) && (greenValue > blueValue)))
            isYellow = true;

        //displays color data to driver
        telemetry.addData("greenValue:  ", greenValue);
        telemetry.addData("redValue: ", redValue);
        telemetry.addData("blueValue: ", blueValue);
        telemetry.addData("is Yellow:", isYellow);
        telemetry.update();*/


        //robot.corral.setPower(0.0);
        lift(DRIVE_SPEED, -.26, 10);
        frontBackDrive(DRIVE_SPEED, -.03, -.03, 2); //front and back wheels
        encoderDrive(DRIVE_SPEED, .1375, .1375, 9); //go from lander to mineral row
        lift(DRIVE_SPEED, .26, 10);
        frontBackDrive(DRIVE_SPEED, -0.07, -0.07, 2);
        encoderDrive(DRIVE_SPEED, .015, .015, 2);

        sleep(1000);
        yellowFound = isYellow();
        if(yellowFound) {
            encoderDrive(DRIVE_SPEED, .25, .25, 9);
            encoderDrive(DRIVE_SPEED, -.1, .1, 9);
            //robot.corral.setPower(0.79);
            sleep(600);
            //robot.corral.setPower(0);
            encoderDrive(DRIVE_SPEED, .025, .025, 9); //deposit marker and go back
        }
        else {
            encoderDrive(DRIVE_SPEED, -.015, -.015, 2); //inch back
            frontBackDrive(DRIVE_SPEED, 0.11, 0.11, 2); //go from one to two
            encoderDrive(DRIVE_SPEED, .0275, .0275, 2); //inch up
            sleep(500);
            yellowFound = isYellow();
            /*if(!isYellow()) {
                encoderDrive(DRIVE_SPEED, -.0275, -.0275, 2); //inch back
                frontBackDrive(DRIVE_SPEED, .01, .01, 2);
                encoderDrive(DRIVE_SPEED, .0275, .0275, 2); //inch up
                sleep(500);
                yellowFound = isYellow();
            }
            if(!isYellow()) {
                encoderDrive(DRIVE_SPEED, -.0275, -.0275, 2); //inch back
                frontBackDrive(DRIVE_SPEED, .01, .01, 2);
                encoderDrive(DRIVE_SPEED, .0275, .0275, 2); //inch up
                sleep(500);
                yellowFound = isYellow();
            }*/
            if (yellowFound) {
                encoderDrive(DRIVE_SPEED, .25, .25, 9);
                encoderDrive(DRIVE_SPEED, -.15, .15, 9);
                //robot.corral.setPower(0.79);
                sleep(600);
                //robot.corral.setPower(0);
                encoderDrive(DRIVE_SPEED, .05, .05, 9); //deposit marker and go back
            }
            else {
                encoderDrive(DRIVE_SPEED, -.025, -.025, 2);
                frontBackDrive(DRIVE_SPEED, 0.1, 0.1, 2);
                encoderDrive(DRIVE_SPEED, .1875, .1875, 9); //knock mineral
                encoderDrive(DRIVE_SPEED, .15, -.15, 9); //turn to depot
                encoderDrive(DRIVE_SPEED, -.1875, -.1875, 9); //go into depot
                //robot.corral.setPower(0.79);
                sleep(600);
                //robot.corral.setPower(0);
                encoderDrive(DRIVE_SPEED, .05, .05, 9); //deposit marker and go back
            }
        }
        /* test for color sensor
        encoderDrive(DRIVE_SPEED, .35, .35,9); //stop in front of mineral
         */
        sleep(500);
    idle();

}

    public void frontBackDrive(double speed, double leftInches, double rightInches, double timeoutS) throws InterruptedException {
        int newLeftTarget; //declare variable for the target position of the left motor
        int newRightTarget; //declare variable for the target position of the right motor

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = robot.frontDrive.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH); //initializes newLeftTarget to represent the desired distance travelled by the left motors
            newRightTarget = robot.backDrive.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH); //initializes newRightTarget to represent the desired distance travelled by the right motors

            //sets the motors to run to the position corresponding to their side (left or right)
            robot.frontDrive.setTargetPosition(newLeftTarget);
            robot.backDrive.setTargetPosition(newRightTarget);


            // Turn On RUN_TO_POSITION
            robot.frontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.backDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            // reset the timeout time and start motion.
            runtime.reset();
            robot.frontDrive.setPower(Math.abs(speed));
            robot.backDrive.setPower(Math.abs(speed));



            // keep looping while still active, and there is time left, and both motors are running.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.backDrive.isBusy() && robot.frontDrive.isBusy())) {

                /*// Display postion on path to the driver.
                telemetry.addData("Path1", "Running to %7d :%7d", newLeftTarget, newRightTarget);
                telemetry.addData("Path2", "Running at %7d :%7d",
                        robot.frontDrive.getCurrentPosition(),
                        robot.backDrive.getCurrentPosition());
                telemetry.update();*/

                // Allow time for other processes to run.
                //idle();
            }

            // Stop all motion;
            robot.backDrive.setPower(0);
            robot.frontDrive.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.backDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.frontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


            sleep(200);   // optional pause after each move
        }
    }

    public void encoderDrive(double speed, double leftInches, double rightInches, double timeoutS) throws InterruptedException {

        int newLeftTarget; //declare variable for the target position of the left motor
        int newRightTarget; //declare variable for the target position of the right motor

        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = robot.leftDrive.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH); //initializes newLeftTarget to represent the desired distance travelled by the left motors
            newRightTarget = robot.rightDrive.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH); //initializes newRightTarget to represent the desired distance travelled by the right motors

            //sets the motors to run to the position corresponding to their side (left or right)
            robot.rightDrive.setTargetPosition(newRightTarget);
            robot.leftDrive.setTargetPosition(newLeftTarget);


            // Turn On RUN_TO_POSITION
            robot.leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            // reset the timeout time and start motion.
            runtime.reset();
            robot.leftDrive.setPower(Math.abs(speed));
            robot.rightDrive.setPower(Math.abs(speed));

            // keep looping while still active, and there is time left, and both motors are running.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.leftDrive.isBusy() && robot.rightDrive.isBusy())) {

                /*// Display postion on path to the driver.
                telemetry.addData("Path1", "Running to %7d :%7d", newLeftTarget, newRightTarget);
                telemetry.addData("Path2", "Running at %7d :%7d",
                        robot.leftDrive.getCurrentPosition(),
                        robot.rightDrive.getCurrentPosition());
                telemetry.update();*/

                // Allow time for other processes to run.
                idle();
            }

            // Stop all motion;
            robot.rightDrive.setPower(0);
            robot.leftDrive.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


            sleep(200);   // optional pause after each move
        }
    }

    public void lift(double speed, double inches, double timeoutS)
    {
        int target;


        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            //target = robot.lift.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH); //initializes target to represent the desired distance travelled by the lift motor

            //sets the motor to run to the position
            //robot.lift.setTargetPosition(target);


            // Turn On RUN_TO_POSITION
            //robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            // reset the timeout time and start motion.
            runtime.reset();
            //robot.lift.setPower(Math.abs(speed));



            // keep looping while still active, and there is time left, and both motors are running.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS)){// &&
                    //(robot.lift.isBusy())) {

                /*// Display postion on path to the driver.
                telemetry.addData("Path1", "Running to %7d", target);
                telemetry.addData("Path2", "Running at %7d", robot.lift.getCurrentPosition());
                telemetry.update();*/

                // Allow time for other processes to run.
                //idle();
            }

            // Stop all motion;
            //robot.lift.setPower(0);


            // Turn off RUN_TO_POSITION
            //robot.lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


            sleep(200);   // optional pause after each move
        }

    }
    public boolean isYellow()
    {
        //variables for different color reading
        //int redValue = robot.colorSensor.red();
        //int greenValue = robot.colorSensor.green();
        //int blueValue = robot.colorSensor.blue();
        boolean isYellow = false;   //jewel is yellow
        //if (((redValue > blueValue * 2) && (greenValue > blueValue)))
        //    isYellow = true;

        //displays color data to driver
        //telemetry.addData("greenValue:  ", greenValue);
        //telemetry.addData("redValue: ", redValue);
        //telemetry.addData("blueValue: ", blueValue);
        telemetry.addData("is Yellow:", isYellow);
        telemetry.update();

        return isYellow;
    }

}

