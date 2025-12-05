package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;

import org.firstinspires.ftc.teamcode.subsystem.Drive;
import org.firstinspires.ftc.teamcode.robot.Global;
import org.firstinspires.ftc.teamcode.subsystem.Sorter;
import org.firstinspires.ftc.teamcode.subsystem.Transfer;
import org.firstinspires.ftc.teamcode.subsystem.Turret;
import org.firstinspires.ftc.teamcode.subsystem.Intake;
import org.firstinspires.ftc.teamcode.robot.Global;
import static org.firstinspires.ftc.teamcode.robot.Global.currentRoom;
import static org.firstinspires.ftc.teamcode.robot.Global.roomIntake;
import static org.firstinspires.ftc.teamcode.robot.Global.roomOuttake;
import static org.firstinspires.ftc.teamcode.robot.Global.artefactsOrder;
import static org.firstinspires.ftc.teamcode.robot.Global.artefacts;

public class Robot{

    public DcMotorEx leftFront;
    public DcMotorEx leftBack;
    public  DcMotorEx rightFront;
    public DcMotorEx rightBack;
    public DcMotorEx intakeMotor; //power
    public DcMotorEx shooterUp
; //power
    public DcMotorEx shooterDown
; //power
    public DcMotorEx sorterMotor; //PID

    public Servo shooterAngle; //range
    public CRServo shooterRotation; //range
    public Servo transferServo; //0-1

    ///Sensors
    public NormalizedColorSensor colorSensor;

    public Drive drive;
    public Turret turret;
    public Sorter sorter;
    public Transfer transfer;
    public Intake intake;

    private static Robot instance = new Robot();
    public boolean enabled;
    public static Robot getInstance(){
        if(instance == null)
            instance = new Robot();
        instance.enabled = true;
        return instance;
    }

    public void init(HardwareMap hardwareMap){
        ///Motors
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

        shooterUp = hardwareMap.get(DcMotorEx.class , "shooterUp ");
        shooterDown = hardwareMap.get(DcMotorEx.class , "shooterDown ");

        shooterUp.setDirection(DcMotorEx.Direction.REVERSE);
        shooterDown.setDirection(DcMotorEx.Direction.FORWARD);

        shooterUp.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        shooterDown.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        sorterMotor = hardwareMap.get(DcMotorEx.class , "sorter");
        sorterMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        sorterMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        sorterMotor.setDirection(DcMotorEx.Direction.FORWARD);
        sorterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intakeMotor = hardwareMap.get(DcMotorEx.class , "intake");
        intakeMotor.setDirection(DcMotorEx.Direction.FORWARD);


        ///Servos
        transferServo = hardwareMap.get(Servo.class , "transfer");
        transferServo.setPosition(0.9);

        shooterRotation = hardwareMap.get(CRServo.class , "shooterRotation");
        shooterRotation.setDirection(DcMotorSimple.Direction.FORWARD);

        ///Sensors
        colorSensor = hardwareMap.get(NormalizedColorSensor.class , "colorSensor");

        ///Globals

        currentRoom=1;
        roomIntake = new int[]{0 ,185 ,370};
        roomOuttake = new int[]{275, 455, 640};
        artefactsOrder= new int[]{-1 ,-1 ,-1};
        artefacts= new int[]{0,0};

        drive = new Drive();
        turret = new Turret();
        sorter = new Sorter();
        transfer = new Transfer();
        intake = new Intake();
    }


}
