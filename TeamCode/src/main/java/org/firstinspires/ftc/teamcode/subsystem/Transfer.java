package org.firstinspires.ftc.teamcode.subsystem;

import org.firstinspires.ftc.teamcode.robot.Robot;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import org.firstinspires.ftc.teamcode.robot.Global;
import static org.firstinspires.ftc.teamcode.robot.Global.currentRoom;
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

    public void StartTransfer()
    {
        button=true;
        timer= new ElapsedTime();
        timer.reset();
        if(robot.intakeMotor.getPower()>0 && robot.shooterUp.getPower()>0 && artefactsOrder[currentRoom-1]>-1) {
            while(button)
                if(timer.seconds()==1)
                {
                    if(artefacts[0]+artefacts[1]>0){
                        robot.transferServo.setPosition(posup);
                        if(artefactsOrder[currentRoom-1]==1)
                            artefacts[1]--;
                        else
                            artefacts[0]--;
                        artefactsOrder[currentRoom-1]=-1;
                    }
                    if(timer.seconds()>2.5)
                    {
                        robot.transferServo.setPosition(posdown);
                        robot.sorter.ChangeRoom(1);
                        timer.reset();
                    }
                    if(artefacts[0]+artefacts[1]==0)
                        button=false;

                }

            }
    }

    public void LaunchStage1(){
        if(robot.intakeMotor.getPower()>0 && robot.shooterUp.getPower()>0 && artefactsOrder[currentRoom-1]>-1){
            RiseTransfer();
            if(artefactsOrder[currentRoom-1]==1)
                artefacts[1]--;
            else
                artefacts[0]--;
            artefactsOrder[currentRoom-1]=-1;
        }
    }
    public void LaunchStage2(){
        LowerTransfer();
        robot.sorter.ChangeRoom(1);
    }

    public boolean EndLaunchSequence(){
        return (artefacts[0]+artefacts[1]==0);
    }

    public void LowerTransfer(){robot.transferServo.setPosition(posdown);}
    public void RiseTransfer(){robot.transferServo.setPosition(posup);}


}
