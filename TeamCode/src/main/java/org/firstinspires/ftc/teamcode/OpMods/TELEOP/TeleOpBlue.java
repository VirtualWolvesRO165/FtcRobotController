package org.firstinspires.ftc.teamcode.OpMods.TELEOP;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import static org.firstinspires.ftc.teamcode.robot.Constants.BASKET_Y;
import static org.firstinspires.ftc.teamcode.robot.Constants.BLUE_BASKET_X;
import static org.firstinspires.ftc.teamcode.robot.Constants.BASKET_X;
import static org.firstinspires.ftc.teamcode.robot.Constants.BLUE_BASKET_Y;
import static org.firstinspires.ftc.teamcode.robot.Constants.ROBOT_X;
import static org.firstinspires.ftc.teamcode.robot.Constants.ROBOT_Y;
import static org.firstinspires.ftc.teamcode.robot.Constants.SHOOTER_RPM;
import static org.firstinspires.ftc.teamcode.robot.Constants.opModType;
import static org.firstinspires.ftc.teamcode.robot.Constants.NOW;
import static org.firstinspires.ftc.teamcode.robot.Constants.Alliance;
import static org.firstinspires.ftc.teamcode.robot.Constants.alliance;
import static org.firstinspires.ftc.teamcode.subsystem.Intake.intakeState;
import static org.firstinspires.ftc.teamcode.subsystem.Intake.stopperState;
import static org.firstinspires.ftc.teamcode.subsystem.Turret.turretRotationState;
import static org.firstinspires.ftc.teamcode.subsystem.Turret.shooterAngleState;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.subsystem.Intake;
import org.firstinspires.ftc.teamcode.subsystem.Turret;

import java.util.function.Supplier;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="TELEOPBLUE")
public class TeleOpBlue extends CommandOpMode {

    private Robot robot = Robot.getInstance();
    private GamepadEx driver;
    private GamepadEx operator;
    private ElapsedTime timer;
    private Follower follower;
    public static Pose startingPose;
    private boolean automatedDrive;
    private Supplier<PathChain> pathChain;


    @Override
    public void initialize(){

        super.reset();
        driver = new GamepadEx(gamepad1); ///creates driver control
        operator = new GamepadEx(gamepad2); ///creates operator control
        alliance = Alliance.BLUE;
        BASKET_X = BLUE_BASKET_X;
        BASKET_Y = BLUE_BASKET_Y;
        ROBOT_X = 56;
        ROBOT_Y=8;
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose == null ? new Pose(56, 8) : startingPose);
        follower.update();

        pathChain = () -> follower.pathBuilder() //Lazy Curve Generation
                .addPath(new Path(new BezierLine(follower::getPose, new Pose(45, 98))))
//                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(45), 0.8))
                .build();

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

        operator.getGamepadButton(GamepadKeys.Button.B).whenHeld(
                new InstantCommand(()->stopperState = Intake.StopperState.OPEN)
        );
        operator.getGamepadButton(GamepadKeys.Button.B).whenReleased(
                new InstantCommand(()->stopperState = Intake.StopperState.CLOSE)
        );

        driver.getGamepadButton(GamepadKeys.Button.X).whenPressed(
                new InstantCommand(()->follower.followPath(pathChain.get()))
        );

        operator.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenHeld(
                new InstantCommand(()->shooterAngleState = Turret.ShooterAngleState.UP)
        );
        operator.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenHeld(
                new InstantCommand(()->shooterAngleState = Turret.ShooterAngleState.DOWN)
        );
        operator.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenReleased(
                new InstantCommand(()->shooterAngleState = Turret.ShooterAngleState.STOP)
        );
        operator.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenReleased(
                new InstantCommand(()->shooterAngleState = Turret.ShooterAngleState.STOP)
        );

        operator.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenPressed(
                new InstantCommand(()->robot.turret.AdjustRPM(-1))
        );
        operator.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(
                new InstantCommand(()->robot.turret.AdjustRPM(1))
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
        follower.update();
        super.run();

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.addData("X" , ROBOT_X);
        telemetry.addData("Y" , ROBOT_Y);
        telemetry.addData("shooterAnglePosition" , robot.shooterAngle.getPosition());
        telemetry.addData("turretRotationState" , turretRotationState);
        telemetry.update();
        timer.reset();
    }


}