package org.firstinspires.ftc.teamcode.OpMods.TELEOP;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.geometry.Pose;

import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import static org.firstinspires.ftc.teamcode.robot.Constants.ANGLE_POSITION;
import static org.firstinspires.ftc.teamcode.robot.Constants.BLUE_BASKET_X;
import static org.firstinspires.ftc.teamcode.robot.Constants.BLUE_BASKET_Y;
import static org.firstinspires.ftc.teamcode.robot.Constants.IS_IN_CLOSE;
import static org.firstinspires.ftc.teamcode.robot.Constants.IS_IN_FAR;
import static org.firstinspires.ftc.teamcode.robot.Constants.OFFSET_TURRET;
import static org.firstinspires.ftc.teamcode.robot.Constants.ROBOT_X;
import static org.firstinspires.ftc.teamcode.robot.Constants.ROBOT_Y;
import static org.firstinspires.ftc.teamcode.robot.Constants.SHOOTER_RPM;
import static org.firstinspires.ftc.teamcode.robot.Constants.SHOOTER_RPM_OFFSET;
import static org.firstinspires.ftc.teamcode.robot.Constants.START_HEADING;
import static org.firstinspires.ftc.teamcode.robot.Constants.NOW;
import static org.firstinspires.ftc.teamcode.robot.Constants.START_POSE;
import static org.firstinspires.ftc.teamcode.subsystem.Intake.stopperState;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.subsystem.Intake;

import java.util.function.Supplier;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="TELEOPBLUE")
public class TeleOpBlue extends CommandOpMode {

    private Robot robot = Robot.getInstance();
    private GamepadEx driver;
    private GamepadEx operator;
    private ElapsedTime timer;

    private int original_turret_offset = OFFSET_TURRET;
    private boolean automatedDrive;
    private Supplier<PathChain> pathChain;

    @Override
    public void initialize(){

        super.reset();
        driver = new GamepadEx(gamepad1); ///creates driver control
        operator = new GamepadEx(gamepad2); ///creates operator control
        robot.init(hardwareMap); ///initialize robot
        START_POSE = new Pose(ROBOT_X , ROBOT_Y);
        robot.limelight.setPollRateHz(100);
        robot.limelight.start();
        robot.limelight.pipelineSwitch(1);
        robot.follower = Constants.createFollower(hardwareMap);
        robot.follower.setStartingPose(START_POSE);
        robot.follower.setHeading(0);
        START_HEADING = Math.toDegrees(robot.follower.getHeading());
        register(robot.drive); ///nush ce face da trebuie
        /// changes state of intake when Y is pressed
//        driver.getGamepadButton(GamepadKeys.Button.Y).whenPressed(
//                new InstantCommand(()->robot.intake.ToggleIntake())
//        );

        /// checks state of shooter when A is pressed
//        operator.getGamepadButton(GamepadKeys.Button.A).whenPressed(
//                new InstantCommand(()->robot.turret.ToggleShooter())
//        );
//
//        operator.getGamepadButton(GamepadKeys.Button.B).whenHeld(
//                new InstantCommand(()->stopperState = Intake.StopperState.OPEN)
//        );
//        operator.getGamepadButton(GamepadKeys.Button.B).whenReleased(
//                new InstantCommand(()->stopperState = Intake.StopperState.CLOSE)
//        );
//        operator.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenHeld(
//                new InstantCommand(()->shooterAngleState = Turret.ShooterAngleState.UP)
//        );
//        operator.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenHeld(
//                new InstantCommand(()->shooterAngleState = Turret.ShooterAngleState.DOWN)
//        );
//        operator.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenReleased(
//                new InstantCommand(()->shooterAngleState = Turret.ShooterAngleState.STOP)
//        );
//        operator.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenReleased(
//                new InstantCommand(()->shooterAngleState = Turret.ShooterAngleState.STOP)
//        );
        operator.getGamepadButton(GamepadKeys.Button.A).whenPressed(
            new InstantCommand(()->robot.turret.ToggleShooter())
        );
//        driver.getGamepadButton(GamepadKeys.Button.X).whenPressed(
//            new InstantCommand(()->robot.robotState = Robot.RobotState.POSITIONING)
//        );
        driver.getGamepadButton(GamepadKeys.Button.Y).whenPressed(
            new InstantCommand(()->robot.intake.ToggleIntake())
        );

        operator.getGamepadButton(GamepadKeys.Button.B).whenHeld(
                new InstantCommand(()->stopperState = Intake.StopperState.OPEN)
        );
        operator.getGamepadButton(GamepadKeys.Button.B).whenReleased(
                new InstantCommand(()->stopperState = Intake.StopperState.CLOSE)
        );

        operator.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(
                new InstantCommand(()->OFFSET_TURRET+=5)
        );
        operator.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).whenPressed(
                new InstantCommand(()->OFFSET_TURRET-=5)
        );
        operator.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(
                new InstantCommand(()->SHOOTER_RPM_OFFSET+=100)
        );
        operator.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(
                new InstantCommand(()->SHOOTER_RPM_OFFSET-=100)
        );
        operator.getGamepadButton(GamepadKeys.Button.Y).whenPressed(
                new InstantCommand(()->OFFSET_TURRET=0)
        );
        super.run();
    }

    /// called every tick
    @Override
    public void run(){
        if(timer==null)
            timer = new ElapsedTime();
        NOW=getRuntime();
        SHOOTER_RPM=robot.turret.FlywheelSpeed(Math.sqrt(Math.pow(BLUE_BASKET_X - ROBOT_X, 2) + Math.pow(BLUE_BASKET_Y - ROBOT_Y, 2)))+SHOOTER_RPM_OFFSET;
        ANGLE_POSITION = robot.turret.shooterAngle(Math.sqrt(Math.pow(BLUE_BASKET_X - ROBOT_X, 2) + Math.pow(BLUE_BASKET_Y - ROBOT_Y, 2)));
        robot.drive.Update(driver.getLeftY(),driver.getLeftX(),driver.getRightX());
        robot.intake.Update(); ///look in subsystem for more info
        robot.turret.Update(); ///look in subsystem for more info
        robot.vision.Update(20);
        robot.Update();
//      robot.intake.CheckIntake();
        robot.turret.AutoAim(BLUE_BASKET_X , BLUE_BASKET_Y , Math.toDegrees(robot.follower.getHeading()));
        super.run();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.addData("X" , robot.follower.getPose().getX());
        telemetry.addData("Y" , robot.follower.getPose().getY());
        telemetry.addData("heading" , Math.toDegrees(robot.follower.getHeading()));
        telemetry.addData("angle" , robot.shooterAngle.getPosition());
        telemetry.addData("shooterRPM" , SHOOTER_RPM);
        telemetry.addData("limelight offset" , robot.vision.Offset());
        telemetry.addData("aprilTag" , robot.vision.AprilTag());
        telemetry.addData("robotState" , robot.robotState);
        telemetry.addData("distanceSensor" , robot.distanceSensor.getDistance(DistanceUnit.CM));
        telemetry.addData("isInFar" , IS_IN_FAR);
        telemetry.addData("isInClose" , IS_IN_CLOSE);
        telemetry.addData("intakeState" , robot.intake.intakeState);
        telemetry.update();
        timer.reset();
    }


}