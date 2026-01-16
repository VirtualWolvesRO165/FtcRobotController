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
import static org.firstinspires.ftc.teamcode.subsystem.Sorter.intakePos;

@Config
public class Transfer extends SubsystemBase {

    private final Robot robot = Robot.getInstance();
    public static double posup=0;
    public static double posdown=1;

    public void LaunchStage1(){
        if(robot.shooterUp.getPower()>0){
            RiseTransfer();
        }
    }
    public void LaunchStage2(){
        LowerTransfer();
        robot.sorter.ChangeRoom(1);
        if(artefactsOrder[currentRoom-1]>-1)
            artefacts--;
        artefactsOrder[currentRoom-1]=-1;
    }

    public boolean EndLaunchSequence(){
        return artefacts==0;
    }


    public void LowerTransfer(){robot.transferServo.setPosition(posdown);}
    public void RiseTransfer(){robot.transferServo.setPosition(posup);}

}
