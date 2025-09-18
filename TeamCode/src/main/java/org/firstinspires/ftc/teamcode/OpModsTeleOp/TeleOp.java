package org.firstinspires.ftc.teamcode.OpModsTeleOp;

import static org.firstinspires.ftc.teamcode.robot.robot.OpModeType.TELEOP;

import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.RunCommand;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;

import org.firstinspires.ftc.teamcode.commands.DriveCommand;
import org.firstinspires.ftc.teamcode.robot.robot;

public class TeleOp extends CommandOpMode {

    private robot robot;
    private GamepadEx gm1;

    @Override
    public void initialize(){
        robot = new robot(telemetry , hardwareMap , TELEOP);
        gm1 = new GamepadEx(gamepad1);

        robot.schedule(
                new RunCommand(robot::read),
                new RunCommand(robot::loop),
                new RunCommand(robot::write)
        );

        robot.drive.setDefaultCommand(new DriveCommand.StickInputs(robot.drive , gm1));

    }


}
