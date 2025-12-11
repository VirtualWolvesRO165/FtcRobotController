package org.firstinspires.ftc.teamcode.commands;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.subsystem.Drive;

public class DriveCommand extends CommandBase {
    private final Robot robot = Robot.getInstance();
    double p_drive;
    double strafe;
    double turn;

    public DriveCommand(double p_drive , double strafe , double turn){
        addRequirements(robot.drive);
        this.p_drive=p_drive;
        this.strafe=strafe;
        this.turn=turn;
    }

    @Override
    public void initialize(){

    }

    public void execute(){
        robot.leftFront.setPower((p_drive+strafe+turn));
        robot.leftBack.setPower((p_drive-strafe+turn));
        robot.rightFront.setPower((p_drive-strafe-turn));
        robot.rightBack.setPower((p_drive+strafe-turn));

    }

    public boolean isFinished(){
        return false;
    }

}
