package org.firstinspires.ftc.teamcode.subsystem;

import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.robot.Robot;

public class Drive extends SubsystemBase
{
    private final Robot robot = Robot.getInstance();
    public double p_drive , strafe, turn;

    public void init(){

    }

    public void PowerMotor(double p_drive , double strafe , double turn){
        robot.leftFront.setPower(p_drive+strafe+turn);
        robot.leftBack.setPower(p_drive-strafe+turn);
        robot.rightFront.setPower(p_drive-strafe-turn);
        robot.rightBack.setPower(p_drive+strafe-turn);
    }


}
