package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;

import org.firstinspires.ftc.teamcode.subsystem.Drive;
import org.firstinspires.ftc.teamcode.robot.Global;
import org.firstinspires.ftc.teamcode.subsystem.Turret;

public class Robot{

    public DcMotorEx leftFront;
    public DcMotorEx leftBack;
    public  DcMotorEx rightFront;
    public DcMotorEx rightBack;
    public DcMotorEx IntakeTransfer; //power
    public DcMotorEx ShooterUp; //power
    public DcMotorEx ShooterDown; //power
    public DcMotorEx Indexer; //PID

    public Servo ShooterAngle; //range
    public Servo ShooterRotation; //range
    public Servo Transfer; //0-1

    public Drive drive;
    public Turret turret;

    private static Robot instance = new Robot();
    public boolean enabled;
    public static Robot getInstance(){
        if(instance == null)
            instance = new Robot();
        instance.enabled = true;
        return instance;
    }

    public void init(HardwareMap hardwareMap){
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

        ShooterUp = hardwareMap.get(DcMotorEx.class , "ShooterUp");
        ShooterDown = hardwareMap.get(DcMotorEx.class , "ShooterDown");

        ShooterUp.setDirection(DcMotorEx.Direction.REVERSE);
        ShooterDown.setDirection(DcMotorSimple.Direction.FORWARD);

        ShooterUp.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        ShooterDown.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        drive = new Drive();
        turret = new Turret();
    }


}
