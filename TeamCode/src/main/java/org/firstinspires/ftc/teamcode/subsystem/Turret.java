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
            robot.shooterUp.setPower(1);
            robot.shooterDown.setPower(1);
            isPowered=true;
        }
        else{
            robot.shooterUp.setPower(0);
            robot.shooterDown.setPower(0);
            isPowered=false;
        }
    }

    public void RotateShooter(double modifier){
        robot.shooterRotation.setPower(modifier);
    }
}
