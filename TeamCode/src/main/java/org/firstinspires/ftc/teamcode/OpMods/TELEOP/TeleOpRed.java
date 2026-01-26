package org.firstinspires.ftc.teamcode.OpMods.TELEOP;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import static org.firstinspires.ftc.teamcode.robot.Constants.BASKET_X;
import static org.firstinspires.ftc.teamcode.robot.Constants.BASKET_Y;
import static org.firstinspires.ftc.teamcode.robot.Constants.RED_BASKET_X;
import static org.firstinspires.ftc.teamcode.robot.Constants.RED_BASKET_Y;
import static org.firstinspires.ftc.teamcode.robot.Constants.opModType;
import static org.firstinspires.ftc.teamcode.robot.Constants.NOW;
import static org.firstinspires.ftc.teamcode.robot.Constants.Alliance;
import static org.firstinspires.ftc.teamcode.robot.Constants.alliance;
import static org.firstinspires.ftc.teamcode.subsystem.Turret.turretRotationState;

import org.firstinspires.ftc.teamcode.robot.Constants;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.subsystem.Turret;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="TELEOPRED")
public class TeleOpRed extends CommandOpMode {

    private Robot robot = Robot.getInstance();
    private GamepadEx driver;
    private GamepadEx operator;
    private ElapsedTime timer;


    @Override
    public void initialize(){

        super.reset();
        driver = new GamepadEx(gamepad1); ///creates driver control
        operator = new GamepadEx(gamepad2); ///creates operator control
        opModType = Constants.OpModTypes.TELEOP; ///select game type
        alliance = Alliance.RED;
        BASKET_X = RED_BASKET_X;
        BASKET_Y = RED_BASKET_Y;
        robot.init(hardwareMap); ///initialize robot
        register(robot.drive); ///nush ce face da trebuie


        /// changes state of intake when Y is pressed
        driver.getGamepadButton(GamepadKeys.Button.Y).whenPressed(
                new InstantCommand(()->robot.intake.ToggleIntake())
        );

        /// checks state of shooter when A is pressed
        operator.getGamepadButton(GamepadKeys.Button.A).whenPressed(
                new InstantCommand(()->robot.turret.ToggleShooter())
        );


        /// moves turret left when left dpad is held
        operator.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenHeld(
                new InstantCommand(()->turretRotationState = Turret.TurretRotationState.LEFT)
        );
        /// moves turret right when right dpad is held
        operator.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).whenHeld(
                new InstantCommand(()->turretRotationState = Turret.TurretRotationState.RIGHT)
        );
        /// stops the movement of turret to the left when left dpad is released
        operator.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenReleased(
                new InstantCommand(()->turretRotationState = Turret.TurretRotationState.STOP)
        );
        /// stops the movement of turret to the right when right dpad is released

        operator.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).whenReleased(
                new InstantCommand(()->turretRotationState = Turret.TurretRotationState.STOP)
        );

        super.run();
    }

    /// called every tick
    @Override
    public void run(){
        if(timer==null)
            timer = new ElapsedTime();
        NOW=getRuntime();
        robot.drive.PowerMotor(driver.getLeftY(),driver.getLeftX(),driver.getRightX());
        robot.intake.Update(); ///look in subsystem for more info
        robot.turret.Update(); ///look in subsystem for more info
        super.run();

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.update();
        timer.reset();
    }

}