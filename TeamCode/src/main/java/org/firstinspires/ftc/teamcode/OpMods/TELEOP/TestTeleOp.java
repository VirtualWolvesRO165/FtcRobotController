package org.firstinspires.ftc.teamcode.OpMods.TELEOP;

import static org.firstinspires.ftc.teamcode.robot.Constants.ANGLE_POSITION;
import static org.firstinspires.ftc.teamcode.robot.Constants.INTAKE_POWER;
import static org.firstinspires.ftc.teamcode.robot.Constants.SHOOTER_RPM;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.robot.Robot;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="TestTeleOp")
public class TestTeleOp extends CommandOpMode {

    private Robot robot = Robot.getInstance();
    private GamepadEx driver;
    private GamepadEx operator;
    private ElapsedTime timer;
    @Override
    public void initialize(){

        super.reset();
        driver = new GamepadEx(gamepad1); ///creates driver control
        operator = new GamepadEx(gamepad2); ///creates operator control
        robot.init(hardwareMap); ///initialize robot
        register(robot.drive); ///nush ce face da trebuie

        driver.getGamepadButton(GamepadKeys.Button.Y).whenPressed(
                new InstantCommand(()->robot.intakeMotor.setPower(-INTAKE_POWER))
        );
        driver.getGamepadButton(GamepadKeys.Button.X).whenPressed(
                new InstantCommand(()->robot.intakeMotor.setPower(0))
        );
        driver.getGamepadButton(GamepadKeys.Button.B).whenPressed(
                new InstantCommand(()->robot.stopper.setPosition(0))
        );
        driver.getGamepadButton(GamepadKeys.Button.A).whenPressed(
                new InstantCommand(()->robot.stopper.setPosition(1))
        );


        super.run();
    }

    /// called every tick
    @Override
    public void run(){
        robot.shooterUp.setVelocity(SHOOTER_RPM);
        robot.shooterDown.setVelocity(SHOOTER_RPM);
        robot.shooterAngle.setPosition(ANGLE_POSITION);
        telemetry.addData("shooterRPM" , robot.shooterUp.getVelocity());
        telemetry.addData("shooterRPM" , robot.shooterDown.getVelocity());
        if(timer==null)
            timer = new ElapsedTime();
        super.run();telemetry.update();
       timer.reset();
    }


}