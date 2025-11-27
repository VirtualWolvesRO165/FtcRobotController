package org.firstinspires.ftc.teamcode.subsystem;

import org.firstinspires.ftc.teamcode.robot.Robot;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.SubsystemBase;

@Config
public class Transfer extends SubsystemBase {

    private final Robot robot = Robot.getInstance();
    ElapsedTime timer = new ElapsedTime();
    public static double posup=0;
    public static double posdown=0.9;
    private boolean up=false , button=false;

    public void StartTransfer()
    {
        up=!up;
        if(!up){
            robot.transferServo.setPosition(posup);
        }
        else{
            robot.transferServo.setPosition(posdown);
        }

    }


}
