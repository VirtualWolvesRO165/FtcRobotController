package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.controller.PIDController;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.Robot;

import static org.firstinspires.ftc.teamcode.robot.Global.artefacts;
import static org.firstinspires.ftc.teamcode.robot.Global.artefactsOrder;
import  static org.firstinspires.ftc.teamcode.robot.Global.currentRoom;
import static org.firstinspires.ftc.teamcode.robot.Global.roomIntake;
import static org.firstinspires.ftc.teamcode.robot.Global.roomOuttake;

import android.graphics.Color;

@Config
public class Sorter extends SubsystemBase {

    private final Robot robot = Robot.getInstance();
    ///SorterMotor
    private PIDController controller;
    public static double kp=0.0025,ki=0,kd=0,kf=0;
    public static int sorterTarget=0;
    public static int sorterPos = 0;
    public static double power=0.7 , lowpower=0.3;
    private boolean intakePos=true;
    private static int difference=0;
    private final double ticks_in_degrees = 700/180.0;

    ///ColorSensor
    public static float gain;
    public static NormalizedRGBA colors;
    public static final float[] hsvValues = new float[3];
    private boolean roomMoved=false;
    private boolean roomFull=false;


    public void init(){
        artefacts[0]=0;
        artefacts[1]=0;
        if (robot.colorSensor instanceof SwitchableLight) {
            ((SwitchableLight)robot.colorSensor).enableLight(true);
        }
    }


    ///SorterMotor
    public void loop(){
        sorterPos=robot.sorterMotor.getCurrentPosition();
//        controller = new PIDController(kp , ki , kd);
//        controller.setPID(kp , ki , kd);
//        double pid = controller.calculate(sorterPos,sorterTarget);
        robot.sorterMotor.setTargetPosition(sorterTarget);
        robot.sorterMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        if(Math.abs(sorterTarget-sorterPos)<=difference)
            robot.sorterMotor.setPower(lowpower);
        else
            robot.sorterMotor.setPower(power);
    }

    public void SetTarget(int target){
        sorterTarget = target;
    }

    public void RoomTrigger(){
        if(!intakePos){
            SetTarget(roomIntake[currentRoom-1]);
            intakePos=true;
        }
        else{
            SetTarget(roomOuttake[currentRoom-1]);
            intakePos=false;
        }
    }

    public void ChangeRoom(int n){
        currentRoom+=n;
        if(currentRoom>3)
            currentRoom=1;
        if(currentRoom<1)
            currentRoom=3;
        intakePos=!intakePos;
        RoomTrigger();
    }

    ///ColorSensor
    public void ColorDetection(){
        colors = robot.colorSensor.getNormalizedColors();
        Color.colorToHSV(colors.toColor(), hsvValues);
        if(hsvValues[0]>=150 && hsvValues[0]<=200 && !roomMoved)
        {
            if(artefacts[0]+artefacts[1]<2)
                ChangeRoom(1);
            artefacts[0]++;
            artefactsOrder[currentRoom-1]=0;
            roomMoved=true;
        }
        if(hsvValues[0]>200 && hsvValues[0]<=300 && !roomMoved)
        {
            if(artefacts[0]+artefacts[1]<2)
                ChangeRoom(1);
            artefacts[1]++;
            artefactsOrder[currentRoom-1]=1;
            roomMoved=true;
        }
        if(hsvValues[0]<100 && roomMoved)
            roomMoved=false;
        if(artefacts[0]+artefacts[1]<3)
            roomFull=false;
        if(artefacts[0]+artefacts[1]==3 && !roomFull){
            RoomTrigger();
            roomFull=true;
        }
    }

}
