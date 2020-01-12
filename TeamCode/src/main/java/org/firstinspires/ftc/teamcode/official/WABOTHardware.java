package org.firstinspires.ftc.teamcode.official;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

public class WABOTHardware {
    private HardwareMap hardwareMap;

    final double LEFTARMSERVO_IN = 0.5;
    final double LEFTARMSERVO_OUT = 0.7;
    final double RIGHTARMSERVO_IN = 0.8;
    final double RIGHTARMSERVO_OUT = 0.5;

    final double LEFTFOUND_DOWN = 1f;
    final double LEFTFOUND_UP = 0.5f;
    final double RIGHTFOUND_DOWN = 0.5f;
    final double RIGHTFOUND_UP = 1f;

    final double FRONTARMARMSERVO_IN = 0.5;
    final double FRONTARMARMSERVO_OUT = 1;

    final double BACKARMSERVO_IN = 0.9;
    final double BACKARMSERVO_OUT = 0;

    final double CAPSERVO_IN = 0.9;
    final double CAPSERVO_OUT = 0.4; //todo find positions

    public DcMotor FLMotor;
    public DcMotor FRMotor;
    public DcMotor BLMotor;
    public DcMotor BRMotor;
    public DcMotor LIntake;
    public DcMotor RIntake;
    public Servo leftFound;
    public Servo rightFound;
    public DcMotor slideArm; //horizontal slide
    public Servo LArmServo; //stone grabber servos
    public Servo RArmServo; //stone grabber servo
    public DcMotor liftMotor; //vertical slides
    public Servo capServo;
    public DcMotor leftEncoder;
    public DcMotor rightEncoder;
    //public DcMotor strafeEncoder;

//hi owen
//you're adopted
    protected WABOTHardware(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
        initializeMap();
    }
    private void initializeMap() {
        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        FLMotor = hardwareMap.get(DcMotor.class, "FLMotor");
        FRMotor = hardwareMap.get(DcMotor.class, "FRMotor");
        BLMotor = hardwareMap.get(DcMotor.class, "BLMotor");
        BRMotor = hardwareMap.get(DcMotor.class, "BRMotor");
        slideArm = hardwareMap.get(DcMotor.class, "slideArm");
        liftMotor = hardwareMap.get(DcMotor.class, "liftMotor");
        LIntake = hardwareMap.get(DcMotor.class, "LIntake");
        RIntake = hardwareMap.get(DcMotor.class, "RIntake");
        LArmServo = hardwareMap.get(Servo.class, "LArmServo");
        RArmServo = hardwareMap.get(Servo.class, "RArmServo");
        leftFound = hardwareMap.get(Servo.class, "leftFound");
        rightFound = hardwareMap.get(Servo.class, "rightFound");
        capServo = hardwareMap.get(Servo.class, "capServo");
        //leftEncoder = hardwareMap.get(DcMotor.class, "leftEncoder");
        //rightEncoder = hardwareMap.get(DcMotor.class, "rightEncoder");




        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        BRMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        BLMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        FRMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        FLMotor.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public double getRightEncoderPos(){
        return RIntake.getCurrentPosition();
    }

    public double getLeftEncoderPos(){
        return slideArm.getCurrentPosition();
    }

    public double getStrafeEncoderPos(){
        return LIntake.getCurrentPosition();
    }
}