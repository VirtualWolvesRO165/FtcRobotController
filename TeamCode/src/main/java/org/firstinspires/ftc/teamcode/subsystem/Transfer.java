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
    ElapsedTime timer = new ElapsedTime();
    public static double posup=0;
    public static double posdown=0.9;
    private boolean up=false , button=false;

    public void StartTransfer()
    {
        up=!up;
        if(!up){
            if(robot.intakeMotor.getPower()>0 && robot.shooterUp.getPower()>0 && artefactsOrder[currentRoom-1]>-1) {
                robot.transferServo.setPosition(posup);
                if(artefactsOrder[currentRoom-1]==1)
                    artefacts[1]--;
                else
                    artefacts[0]--;
                artefactsOrder[currentRoom-1]=-1;

            }
        }
        else{
            robot.transferServo.setPosition(posdown);
        }

    }


}
