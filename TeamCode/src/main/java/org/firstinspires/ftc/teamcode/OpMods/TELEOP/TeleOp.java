package org.firstinspires.ftc.teamcode.OpMods.TELEOP;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;

import org.firstinspires.ftc.teamcode.robot.Robot;

public class TeleOp extends CommandOpMode {

    private Robot robot = Robot.getInstance();
    private GamepadEx driver;
    private GamepadEx operator;
    private ElapsedTime timer;

    @Override
    public void initialize(){

        super.reset();
        driver = new GamepadEx(gamepad1);
        robot.init(hardwareMap);
        register(robot.drive);
        super.run();
    }

    @Override
    public void run(){
        if(timer==null)
            timer = new ElapsedTime();

        super.run();
        telemetry.update();
        timer.reset();
    }

}
