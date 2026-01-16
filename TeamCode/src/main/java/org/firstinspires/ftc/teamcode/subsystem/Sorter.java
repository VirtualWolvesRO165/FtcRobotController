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
import static org.firstinspires.ftc.teamcode.robot.Global.outtaking;
import static org.firstinspires.ftc.teamcode.robot.Global.roomIntake;
import static org.firstinspires.ftc.teamcode.robot.Global.roomOuttake;

import android.graphics.Color;

@Config
public class Sorter extends SubsystemBase {

    private final Robot robot = Robot.getInstance();
    ///SorterMotor
    public static int sorterTarget=0;
    public static int sorterPos = 0;
    public static double power=0.7;
    public static boolean intakePos=true;
    private final double ticks_in_degrees = 700/180.0;

    ///ColorSensor
    public static float gain;
    public static NormalizedRGBA colors;
    public static final float[] hsvValues = new float[3];
    private boolean roomMoved=false;
    private boolean roomFull=false;


    public void init(){
        outtaking=false;
        artefacts=0;
        sorterTarget=0;
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
            if(artefacts<3){
                ChangeRoom(1);
                artefacts++;
                artefactsOrder[currentRoom-1]=0;
                roomMoved=true;
            }
        }
        if(hsvValues[0]>200 && hsvValues[0]<=300 && !roomMoved)
        {
            if(artefacts<3)
            {
                ChangeRoom(1);
                artefacts++;
                artefactsOrder[currentRoom-1]=1;
                roomMoved=true;
            }
        }
        if(hsvValues[0]<100 && roomMoved)
            roomMoved=false;
        if(artefacts<3 && !outtaking)
            roomFull=false;
        if(artefacts==3 && !roomFull){
            RoomTrigger();
            roomFull=true;
        }
    }

    public void ResetSorter(){
        artefacts=0;
        currentRoom=1;
        artefactsOrder = new int[]{-1,-1,-1};
        sorterTarget=0;
        intakePos=false;
        roomFull=false;
        outtaking=false;
        robot.transfer.LowerTransfer();
    }

    public void SorterOverwrite(){
        sorterTarget=sorterPos;
    }
}
