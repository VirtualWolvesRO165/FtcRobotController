package org.firstinspires.ftc.teamcode.commands;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.subsystem.Drive;

public class DriveCommand extends CommandBase {
    private final Robot robot;

    public DriveCommand(Robot robot){
        this.robot = robot;
        addRequirements(robot.drive);
    }

    @Override
    public void initialize(){

    }

}
