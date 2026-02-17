package org.firstinspires.ftc.teamcode.robot;

import static org.firstinspires.ftc.teamcode.robot.Constants.CAN_SHOOT;
import static org.firstinspires.ftc.teamcode.robot.Constants.IS_BEFORE_IN_CLOSE;
import static org.firstinspires.ftc.teamcode.robot.Constants.IS_BEFORE_IN_FAR;
import static org.firstinspires.ftc.teamcode.robot.Constants.IS_FULL;
import static org.firstinspires.ftc.teamcode.robot.Constants.IS_IN_CLOSE;
import static org.firstinspires.ftc.teamcode.robot.Constants.IS_IN_FAR;
import static org.firstinspires.ftc.teamcode.robot.Constants.SHOOTER_RPM;
import static org.firstinspires.ftc.teamcode.robot.Constants.START_HEADING;
import static org.firstinspires.ftc.teamcode.robot.Constants.START_POSE;

import com.pedropathing.follower.Follower;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystem.Drive;
import org.firstinspires.ftc.teamcode.subsystem.Turret;
import org.firstinspires.ftc.teamcode.subsystem.Intake;
import org.firstinspires.ftc.teamcode.subsystem.Vision;

public class Robot{

    ///DRIVE
    public DcMotorEx leftFront;
    public DcMotorEx leftBack;
    public  DcMotorEx rightFront;
    public DcMotorEx rightBack;
    public Follower follower;
    public IMU imu;

    ///INTAKE
    public DcMotorEx intakeMotor; //power
    public Servo dropDownServoLeft;
    public Servo dropDownServoRight;
    public Servo stopper;
    public DistanceSensor distanceSensor;
    public DistanceSensor distanceSensor2;

    ///TURRET
    public DcMotorEx shooterUp; //power
    public DcMotorEx shooterDown; //power
    public Servo shooterAngle; //range
    public DcMotorEx shooterRotation; //range

    ///AUX
    public NormalizedColorSensor colorSensor;
    public VoltageSensor batteryVoltage;



    ///VISION
    public Limelight3A limelight;

    ///PINPOINT
    public GoBildaPinpointDriver pinpoint;

    public Drive drive;
    public Turret turret;
    public Intake intake;
    public Vision vision;
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

        imu = hardwareMap.get(IMU.class , "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD,
                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);

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

        batteryVoltage = hardwareMap.voltageSensor.iterator().next();

        ///INTAKE
        intakeMotor = hardwareMap.get(DcMotorEx.class , "intake");
        intakeMotor.setDirection(DcMotorEx.Direction.FORWARD);

        dropDownServoLeft = hardwareMap.get(Servo.class , "dropDownLeft");
        dropDownServoRight = hardwareMap.get(Servo.class , "dropDownRight");

        dropDownServoLeft.setDirection(Servo.Direction.REVERSE);
        dropDownServoRight.setDirection(Servo.Direction.FORWARD);

        stopper = hardwareMap.get(Servo.class , "stopper");

//        ///SENSORS
//        distanceSensor = hardwareMap.get(DistanceSensor.class , "distanceSensor");
//        distanceSensor2 = hardwareMap.get(DistanceSensor.class , "distanceSensor2");

        ///VISION
        limelight = hardwareMap.get(Limelight3A.class , "limelight");
        limelight.pipelineSwitch(1);

        ///PINPOINT
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class , "pinpoint");

        stopper.setPosition(1);
        shooterAngle.setPosition(0);

        drive = new Drive();
        turret = new Turret();
        intake = new Intake();
        vision = new Vision();
    }

    public void initAuto(HardwareMap hardwareMap){
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

        batteryVoltage = hardwareMap.voltageSensor.iterator().next();


        ///INTAKE
        intakeMotor = hardwareMap.get(DcMotorEx.class , "intake");
        intakeMotor.setDirection(DcMotorEx.Direction.FORWARD);

        dropDownServoLeft = hardwareMap.get(Servo.class , "dropDownLeft");
        dropDownServoRight = hardwareMap.get(Servo.class , "dropDownRight");

        dropDownServoLeft.setDirection(Servo.Direction.REVERSE);
        dropDownServoRight.setDirection(Servo.Direction.FORWARD);

        stopper = hardwareMap.get(Servo.class , "stopper");

        ///SENSORS
//        distanceSensor = hardwareMap.get(DistanceSensor.class , "distanceSensor");
//        distanceSensor2 = hardwareMap.get(DistanceSensor.class , "distanceSensor2");

        ///VISION
        limelight = hardwareMap.get(Limelight3A.class , "limelight");
        limelight.pipelineSwitch(1);

        ///PINPOINT
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class , "pinpoint");

        stopper.setPosition(1);
        shooterAngle.setPosition(0);

        drive = new Drive();
        turret = new Turret();
        intake = new Intake();
        vision = new Vision();
    }

    public void Update(){
//        if(IS_BEFORE_IN_FAR || IS_BEFORE_IN_CLOSE){
//            turret.StartShooter();
//        }
//
//        if(IS_FULL && !IS_IN_FAR && !IS_IN_CLOSE){
//            intake.StopIntake();
//        }
//
//        if(!IS_FULL)
//            intake.StartIntake();
//
//        if((IS_IN_CLOSE || IS_IN_FAR) && Math.abs(shooterUp.getVelocity()-SHOOTER_RPM)<50){
//            CAN_SHOOT=true;
//            intake.StartIntake();
//        }
//        else
//            CAN_SHOOT=false;
//
//        if(!IS_IN_FAR && !IS_BEFORE_IN_FAR && !IS_IN_CLOSE && !IS_BEFORE_IN_CLOSE)
//            turret.StopShooter();
    }


}
