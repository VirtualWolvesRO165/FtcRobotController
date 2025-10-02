package org.firstinspires.ftc.teamcode.OpMods.TELEOP;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.RunCommand;
import com.seattlesolvers.solverslib.command.button.Trigger;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.robot.Robot;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="1TELEOP")
public class TeleOp extends CommandOpMode {

    private Robot robot = Robot.getInstance();
    private GamepadEx driver;
    private GamepadEx operator;
    private ElapsedTime timer;

    @Override
    public void initialize(){

        super.reset();
        driver = new GamepadEx(gamepad1);
        operator = new GamepadEx(gamepad2);
        robot.init(hardwareMap);
        register(robot.drive);
        robot.drive.setDefaultCommand(
                new RunCommand(()->{
                    robot.drive.PowerMotor(driver.getLeftY() , driver.getLeftX() , driver.getRightX());
                })
        );
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
