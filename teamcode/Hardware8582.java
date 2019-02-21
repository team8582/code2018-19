package org.firstinspires.ftc.teamcode.teamcode;


import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;


public class Hardware8582 {
    public DcMotor leftDrive = null;
    public DcMotor rightDrive = null;
    public DcMotor frontDrive = null;
    public DcMotor backDrive= null;


    public DcMotor lift = null;

    //public CRServo corral = null;


    HardwareMap hwMap8582 = null;


    private ElapsedTime period = new ElapsedTime();

    public Hardware8582(){

    }

    public void init(HardwareMap ahwMap) {
        hwMap8582 = ahwMap;

        leftDrive = hwMap8582.get(DcMotor.class, "leftDrive");
        rightDrive = hwMap8582.get(DcMotor.class, "rightDrive");
        frontDrive = hwMap8582.get(DcMotor.class, "frontDrive");
        backDrive = hwMap8582.get(DcMotor.class, "backDrive");
        lift = hwMap8582.get(DcMotor.class, "lift");

        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);
        frontDrive.setDirection(DcMotor.Direction.REVERSE);
        backDrive.setDirection(DcMotor.Direction.FORWARD);

        lift.setDirection(DcMotor.Direction.REVERSE);

        //corral = hwMap8582.get(CRServo.class, "corral");
        //corral.setPower(0.0);

    }

    /***
     * waitForTick implements a periodic delay. However, this acts like a metronome with a regular
     * periodic tick.  This is used to compensate for varying processing times for each cycle.
     * The function looks at the elapsed cycle time, and sleeps for the remaining time interval.
     *
     * @param periodMs Length of wait cycle in mSec.
     * @throws InterruptedException
     */
    public void waitForTick (long periodMs) throws InterruptedException {

        long remaining = periodMs - (long)period.milliseconds();
        if (remaining > 0)
            Thread.sleep(remaining);
        period.reset();
    }
}
