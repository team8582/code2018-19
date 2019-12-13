package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.internal.opmode.TelemetryImpl;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="AutoTest")
public class AutoTest extends LinearOpMode {
    Hardware8582 robot = new Hardware8582();
    private ElapsedTime runtime = new ElapsedTime();       //Create variable to keep track of elapsed time.
    static final double DRIVE_SPEED = 0.7;
    static final double TURN_SPEED = 0.5;
    final double COUNTS_PER_MOTOR_REV = 4;     //REV Core Hex Motor
    final double DRIVE_GEAR_REDUCTION = 1.0;      //This is < 1.0 if geared up
    final double WHEEL_DIAMETER_INCHES = 3.75;     //For figuring circumference
    final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) *
            (WHEEL_DIAMETER_INCHES * 3.1415);
    int skystoneCount = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);

        //robot.color.enableLed(true); //turn on LED which causes jewel to reflect wavelengths for the sensor to read

        //turn on encoders
        robot.leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.liftTop.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        idle();

        robot.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.frontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.backDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.liftTop.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        while (opModeIsActive()) {

            /*//variables for different color reading
            int redValue = robot.color.red();
            int greenValue = robot.color.green();
            int blueValue = robot.color.blue();
            boolean isYellow = ((redValue > blueValue + 10) && (greenValue > blueValue));   //jewel is yellow


            //displays color data to driver
            telemetry.addData("greenValue:  ", greenValue);
            telemetry.addData("redValue: ", redValue);
            telemetry.addData("blueValue: ", blueValue);
            telemetry.addData("is Yellow:", isYellow);*/
            telemetry.update();

            double OPEN = -1.0;     //left servo open position
            final double CLOSE = .9;   //left servo closed position


            //telemetry.addData("range", String.format("%.01f in", robot.dist.getDistance(DistanceUnit.INCH)));

            lift(DRIVE_SPEED, -2, 1);
            robot.liftServo.setPosition(OPEN);

            while(opModeIsActive() && robot.dist.getDistance(DistanceUnit.INCH)>12)
            {
                forwardBackwardDrive(DRIVE_SPEED, -.3, -.33, 1);
            }
            forwardBackwardDrive(DRIVE_SPEED, -.2, -.2, 1);
            /*lift(DRIVE_SPEED, 2, 1);
            sleep(1000);
            robot.liftServo.setPosition(CLOSE);
            sleep(1000);
            lift(DRIVE_SPEED, -1, 1);
            sleep(1000);
            break;*/

            while(opModeIsActive() && skystoneCount<1)
            {
                if(robot.dist.getDistance(DistanceUnit.INCH)<12 && isYellow())
                {
                    sidewaysDrive(DRIVE_SPEED, .2,.2, 1);
                    telemetry.addData("is Yellow:", isYellow());
                    telemetry.update();
                }
                else if(robot.dist.getDistance(DistanceUnit.INCH)<12 && !isYellow()) {
                    skystoneCount++;
                    lift(DRIVE_SPEED, 2, 1);
                    sleep(1000);
                    robot.liftServo.setPosition(CLOSE);
                    //lift(DRIVE_SPEED,-1, 1);
                    //forwardBackwardDrive(DRIVE_SPEED, -2,-2,2);
                    //lift(DRIVE_SPEED, 1, 1);
                    //robot.liftServo.setPosition(CLOSE);
                    //lift(DRIVE_SPEED, -1,1);
                    //forwardBackwardDrive(DRIVE_SPEED,-1,-1,2); // drive directly forward OR use tick count to go back to beginning???
                    //sidewaysDrive(DRIVE_SPEED, -1,-1,2); //strafe into build site
                    //forwardBackwardDrive(DRIVE_SPEED,1,-1,.5); //turn 180
                    //forwardBackwardDrive(DRIVE_SPEED,-.2,-.2,.5); //inch forward
                    //robot.liftServo.setPosition(OPEN); //drop block
                    //sidewaysDrive(DRIVE_SPEED,1,1,.5); //strafe to back wall
                    //robot.liftServo.setPosition(CLOSE);
                    //lift(DRIVE_SPEED,1,1); //arm back down
                    //forwardBackwardDrive(DRIVE_SPEED,1,1,0.5); //pull foundation
                    break;
                }
            }

        }
        robot.frontDrive.setPower(0);
        robot.backDrive.setPower(0);
        robot.leftDrive.setPower(0);
        robot.rightDrive.setPower(0);
        robot.liftTop.setPower(0);
        idle();
    }

    public void sidewaysDrive(double speed, double leftInches, double rightInches, double timeoutS) throws InterruptedException {
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


    public void forwardBackwardDrive(double speed, double leftInches, double rightInches, double timeoutS) throws InterruptedException {

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

    public void lift(double speed, double inches, double timeoutS) {
        int target;


        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            target = robot.liftTop.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH); //initializes target to represent the desired distance travelled by the lift motor

            //sets the motor to run to the position
            robot.liftTop.setTargetPosition(target);


            // Turn On RUN_TO_POSITION
            robot.liftTop.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            // reset the timeout time and start motion.
            runtime.reset();
            robot.liftTop.setPower(Math.abs(speed));


            // keep looping while still active, and there is time left, and both motors are running.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.liftTop.isBusy())) {

                /*// Display postion on path to the driver.
                telemetry.addData("Path1", "Running to %7d", target);
                telemetry.addData("Path2", "Running at %7d", robot.lift.getCurrentPosition());
                telemetry.update();*/

                // Allow time for other processes to run.
                //idle();
            }

            // Stop all motion;
            robot.liftTop.setPower(0);


            // Turn off RUN_TO_POSITION
            robot.liftTop.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


            sleep(200);   // optional pause after each move
        }
    }

    public boolean isYellow()
    {
        //variables for different color reading
        int redValue = robot.color.red();
        int greenValue = robot.color.green();
        int blueValue = robot.color.blue();
        boolean isYellow = ((redValue > blueValue + 10) && (greenValue > blueValue));   //jewel is yellow
        return isYellow;
    }

}
