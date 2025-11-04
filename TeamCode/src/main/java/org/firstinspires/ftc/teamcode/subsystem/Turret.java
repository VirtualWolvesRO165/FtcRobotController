package org.firstinspires.ftc.teamcode.subsystem;

import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.robot.Robot;

public class Turret extends SubsystemBase {

    private final Robot robot = Robot.getInstance();

    private boolean isPowered=false;

    public void init(){

    }

    public void ShooterPower(){
        if(!isPowered)
        {
            robot.ShooterUp.setPower(1);
            robot.ShooterDown.setPower(1);
            isPowered=true;
        }
        else{
            robot.ShooterUp.setPower(0);
            robot.ShooterDown.setPower(0);
            isPowered=false;
        }
    }
}
