package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.subsystem.Drive;
import org.firstinspires.ftc.teamcode.subsystem.Turret;
import org.firstinspires.ftc.teamcode.subsystem.Intake;

public class Robot{

    ///DRIVE
    public DcMotorEx leftFront;
    public DcMotorEx leftBack;
    public  DcMotorEx rightFront;
    public DcMotorEx rightBack;

    ///INTAKE
    public DcMotorEx intakeMotor; //power
    public Servo dropDownServoLeft;
    public Servo dropDownServoRight;
    public Servo stopper;

    ///TURRET
    public DcMotorEx shooterUp; //power
    public DcMotorEx shooterDown; //power
    public Servo shooterAngle; //range
    public DcMotorEx shooterRotation; //range

    ///Sensors
    public NormalizedColorSensor colorSensor;

    ///VISION
    public Limelight3A limelight;

    ///PINPOINT
    public GoBildaPinpointDriver pinpoint;

    public Drive drive;
    public Turret turret;
    public Intake intake;

    private static Robot instance = new Robot();
    public boolean enabled;
    public static Robot getInstance(){
        if(instance == null)
            instance = new Robot();
        instance.enabled = true;
        return instance;
    }

    public enum RobotState{
        SEARCHING, /// DRIVE-ENABLED , TURRET-DISABLED , INTAKE-ENABLED
        POSITIONING , /// DRIVE-ENABLED , TURRET-DISABLED , INTAKE-DISABLED
        SHOOTING , /// DRIVE-DISABLED , TURRET-ENABLED , INTAKE-ENABLED
    }

    public static RobotState robotState = RobotState.SEARCHING;

    public void init(HardwareMap hardwareMap){
        ///DRIVE
        leftFront = hardwareMap.get(DcMotorEx.class , "leftFront");
        leftBack = hardwareMap.get(DcMotorEx.class , "leftBack");
        rightFront = hardwareMap.get(DcMotorEx.class , "rightFront");
        rightBack = hardwareMap.get(DcMotorEx.class , "rightBack");

        leftBack.setDirection(DcMotorEx.Direction.REVERSE);
        leftFront.setDirection(DcMotorEx.Direction.REVERSE);

        leftFront.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        ///TURRET
        shooterUp = hardwareMap.get(DcMotorEx.class , "shooterUp ");
        shooterDown = hardwareMap.get(DcMotorEx.class , "shooterDown ");

        shooterUp.setDirection(DcMotorEx.Direction.FORWARD);
        shooterDown.setDirection(DcMotorEx.Direction.REVERSE);

        shooterUp.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        shooterDown.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        shooterRotation = hardwareMap.get(DcMotorEx.class , "shooterRotation");
        shooterRotation.setDirection(DcMotorEx.Direction.FORWARD);
        shooterRotation.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        shooterAngle = hardwareMap.get(Servo.class , "shooterAngle");

        ///INTAKE
        intakeMotor = hardwareMap.get(DcMotorEx.class , "intake");
        intakeMotor.setDirection(DcMotorEx.Direction.FORWARD);

        dropDownServoLeft = hardwareMap.get(Servo.class , "dropDownLeft");
        dropDownServoRight = hardwareMap.get(Servo.class , "dropDownRight");

        dropDownServoLeft.setDirection(Servo.Direction.REVERSE);
        dropDownServoRight.setDirection(Servo.Direction.FORWARD);

        stopper = hardwareMap.get(Servo.class , "stopper");

        ///SENSORS
        colorSensor = hardwareMap.get(NormalizedColorSensor.class , "colorSensor");

        ///VISION
//        limelight = hardwareMap.get(Limelight3A.class , "limelight");

        ///PINPOINT
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class , "pinpoint");

        stopper.setPosition(1);
        shooterAngle.setPosition(0);

        drive = new Drive();
        turret = new Turret();
        intake = new Intake();
    }

    public void initAuto(HardwareMap hardwareMap){
        ///TURRET
        shooterUp = hardwareMap.get(DcMotorEx.class , "shooterUp ");
        shooterDown = hardwareMap.get(DcMotorEx.class , "shooterDown ");

        shooterUp.setDirection(DcMotorEx.Direction.FORWARD);
        shooterDown.setDirection(DcMotorEx.Direction.REVERSE);

        shooterUp.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        shooterDown.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        shooterUp.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooterDown.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        shooterRotation = hardwareMap.get(DcMotorEx.class , "shooterRotation");
        shooterRotation.setDirection(DcMotorEx.Direction.FORWARD);
        shooterRotation.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        shooterAngle = hardwareMap.get(Servo.class , "shooterAngle");

        ///INTAKE
        intakeMotor = hardwareMap.get(DcMotorEx.class , "intake");
        intakeMotor.setDirection(DcMotorEx.Direction.FORWARD);

        dropDownServoLeft = hardwareMap.get(Servo.class , "dropDownLeft");
        dropDownServoRight = hardwareMap.get(Servo.class , "dropDownRight");

        dropDownServoLeft.setDirection(Servo.Direction.REVERSE);
        dropDownServoRight.setDirection(Servo.Direction.FORWARD);

        stopper = hardwareMap.get(Servo.class , "stopper");

        ///SENSORS
        colorSensor = hardwareMap.get(NormalizedColorSensor.class , "colorSensor");

        ///VISION
//        limelight = hardwareMap.get(Limelight3A.class , "limelight");

        ///PINPOINT
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class , "pinpoint");

        stopper.setPosition(1);
        shooterAngle.setPosition(0);

        drive = new Drive();
        turret = new Turret();
        intake = new Intake();
    }

    public void Update(){
        switch (robotState){
            case SEARCHING:

        }
    }

}
