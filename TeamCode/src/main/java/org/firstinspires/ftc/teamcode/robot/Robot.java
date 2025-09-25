package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.subsystem.Drive;

public class Robot{

    public DcMotorEx leftFront;
    public DcMotorEx leftBack;
    public  DcMotorEx rightFront;
    public DcMotorEx rightBack;

    public Drive drive;
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

        drive = new Drive();
    }


}
