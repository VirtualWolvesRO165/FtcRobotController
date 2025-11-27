package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.controller.PIDController;

import org.firstinspires.ftc.teamcode.robot.Robot;
import  static org.firstinspires.ftc.teamcode.robot.Global.currentRoom;
import static org.firstinspires.ftc.teamcode.robot.Global.roomIntake;
import static org.firstinspires.ftc.teamcode.robot.Global.roomOuttake;

@Config
public class Sorter extends SubsystemBase {

    private final Robot robot = Robot.getInstance();
    private PIDController controller;
    public static double kp=0,ki=0,kd=0,kf=0;
    public static int sorterTarget=0;
    public static int sorterPos = 0;
    public static double power;
    private boolean intakePos=true;
    private final double ticks_in_degrees = 700/180.0;

    public void init(){
    }

    public void loop(){
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

}
