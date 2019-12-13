package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ColorSensor;


public class Hardware8582 {
    public DcMotor leftDrive = null;
    public DcMotor rightDrive = null;
    public DcMotor frontDrive = null;
    public DcMotor backDrive= null;
    public Servo liftServo = null;
    public DcMotor liftTop = null;
    public ColorSensor color = null;
    public DistanceSensor dist = null;
    //public DcMotor liftBottom = null;

    HardwareMap hwMap8582 = null;


    private ElapsedTime period = new ElapsedTime();

    public Hardware8582(){

    }

    public void init(HardwareMap ahwMap) {
        hwMap8582 = ahwMap;

        //create variables for all motors
        leftDrive = hwMap8582.get(DcMotor.class, "leftDrive");
        rightDrive = hwMap8582.get(DcMotor.class, "rightDrive");
        frontDrive = hwMap8582.get(DcMotor.class, "frontDrive");
        backDrive = hwMap8582.get(DcMotor.class, "backDrive");
        liftServo = hwMap8582.get(Servo.class, "liftServo");
        liftTop = hwMap8582.get(DcMotor.class, "liftTop");
        color = hwMap8582.get(ColorSensor.class, "color");
        dist = hwMap8582.get(DistanceSensor.class, "dist");
        //liftBottom = hwMap8582.get(DcMotor.class, "liftBottom");


        //set direction of dc motors on wheels
        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);
        frontDrive.setDirection(DcMotor.Direction.REVERSE);
        backDrive.setDirection(DcMotor.Direction.FORWARD);


        //set direction of dc motors on lift
        liftTop.setDirection(DcMotor.Direction.REVERSE);
        //liftBottom.setDirection(DcMotor.Direction.REVERSE);


        //set servo position
        liftServo.setPosition(0.9);
    }
}
