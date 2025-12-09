package org.firstinspires.ftc.teamcode.OpMods.TELEOP;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.RepeatCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.command.WaitUntilCommand;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import static org.firstinspires.ftc.teamcode.robot.Global.artefacts;
import static org.firstinspires.ftc.teamcode.robot.Global.artefactsOrder;
import static org.firstinspires.ftc.teamcode.robot.Global.currentRoom;
import static org.firstinspires.ftc.teamcode.robot.Global.outtaking;
import static org.firstinspires.ftc.teamcode.subsystem.Sorter.colors;
import static org.firstinspires.ftc.teamcode.subsystem.Sorter.hsvValues;

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
        robot.sorter.init();
        register(robot.drive);
        operator.getGamepadButton(GamepadKeys.Button.A).whenPressed(
                new InstantCommand(() -> robot.turret.ShooterPower())
        );

        operator.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(
                new InstantCommand(()->{
                    robot.sorter.ChangeRoom(1);
                })
        );
        operator.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(
                new InstantCommand(()->{
                    robot.sorter.ChangeRoom(-1);
                })
        );

        operator.getGamepadButton(GamepadKeys.Button.X).whenPressed(
                new InstantCommand(()->{
                    robot.sorter.RoomTrigger();
                }));
        RepeatCommand launchSequence = new RepeatCommand(
                new SequentialCommandGroup(
                        new WaitCommand(1500),
                        new InstantCommand(()->robot.transfer.LaunchStage1()),
                        new WaitCommand(1000),
                        new InstantCommand(()->robot.transfer.LaunchStage2())
                ),
                ()->robot.transfer.EndLaunchSequence()
        );
        operator.getGamepadButton(GamepadKeys.Button.B).whenPressed(
                new SequentialCommandGroup(
                        launchSequence,
                        new WaitUntilCommand(launchSequence::isFinished),
                        new InstantCommand(()->robot.transfer.LowerTransfer()),
                        new WaitCommand(200),
                        new InstantCommand(()->robot.sorter.RoomTrigger()),
                        new InstantCommand(()->robot.transfer.EndOuttake())
                ));
        operator.getGamepadButton(GamepadKeys.Button.Y).whenPressed(
                new InstantCommand(()->{
                    robot.intake.PowerIntake();
                })
        );


        super.run();
    }

    @Override
    public void run(){
        if(timer==null)
            timer = new ElapsedTime();

        robot.drive.PowerMotor(driver.getLeftY(),driver.getLeftX(),driver.getRightX());
        robot.turret.RotateShooter(operator.getLeftX());
        robot.sorter.loop();
        if(!outtaking)
            robot.sorter.ColorDetection();
        super.run();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.addData("Room" , currentRoom);
        telemetry.addData("SorterPos" , robot.sorter.sorterPos);
        telemetry.addData("Red", "%.3f", colors.red);
        telemetry.addData("Green", "%.3f", colors.green);
        telemetry.addData("Blue", "%.3f", colors.blue);
        telemetry.addData("Hue", "%.3f", hsvValues[0]);
        telemetry.addData("artefacts " , artefactsOrder[0]+" "+artefactsOrder[1]+" "+artefactsOrder[2]);
        telemetry.addData("bile" , artefacts);
        telemetry.addData("bool" , robot.transfer.EndLaunchSequence());
        telemetry.update();
        timer.reset();
    }

}
