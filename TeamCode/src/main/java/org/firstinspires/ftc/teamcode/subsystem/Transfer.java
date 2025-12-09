package org.firstinspires.ftc.teamcode.subsystem;

import org.firstinspires.ftc.teamcode.robot.Robot;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import org.firstinspires.ftc.teamcode.robot.Global;
import static org.firstinspires.ftc.teamcode.robot.Global.currentRoom;
import static org.firstinspires.ftc.teamcode.robot.Global.outtaking;
import static org.firstinspires.ftc.teamcode.robot.Global.roomIntake;
import static org.firstinspires.ftc.teamcode.robot.Global.roomOuttake;
import static org.firstinspires.ftc.teamcode.robot.Global.artefactsOrder;
import static org.firstinspires.ftc.teamcode.robot.Global.artefacts;

@Config
public class Transfer extends SubsystemBase {

    private final Robot robot = Robot.getInstance();
    public static ElapsedTime timer = new ElapsedTime();
    public static double posup=0;
    public static double posdown=0.9;
    private boolean up=false , button=false;

    public void LaunchStage1(){
        if(robot.intakeMotor.getPower()>0 && robot.shooterUp.getPower()>0 && artefactsOrder[currentRoom-1]>-1){
            outtaking=true;
            RiseTransfer();
            if(artefactsOrder[currentRoom-1]>-1)
                artefacts--;
            artefactsOrder[currentRoom-1]=-1;
        }
    }
    public void LaunchStage2(){
        LowerTransfer();
        robot.sorter.ChangeRoom(1);
    }

    public boolean EndLaunchSequence(){
        return (artefacts==0);
    }

    public void EndOuttake(){
        outtaking=false;
    }

    public void LowerTransfer(){robot.transferServo.setPosition(posdown);}
    public void RiseTransfer(){robot.transferServo.setPosition(posup);}


}
