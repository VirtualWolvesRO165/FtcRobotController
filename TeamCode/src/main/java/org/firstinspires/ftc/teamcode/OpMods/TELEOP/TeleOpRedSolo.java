package org.firstinspires.ftc.teamcode.OpMods.TELEOP;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;

import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.robotcore.hardware.SwitchableLight;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import static org.firstinspires.ftc.teamcode.robot.Constants.ANGLE;
import static org.firstinspires.ftc.teamcode.robot.Constants.ANGLE_POSITION;
import static org.firstinspires.ftc.teamcode.robot.Constants.BLUE_BASKET_X;
import static org.firstinspires.ftc.teamcode.robot.Constants.BLUE_BASKET_Y;
import static org.firstinspires.ftc.teamcode.robot.Constants.RED_BASKET_X;
import static org.firstinspires.ftc.teamcode.robot.Constants.RED_BASKET_Y;
import static org.firstinspires.ftc.teamcode.robot.Constants.DRIVE_MULTIPLIER;
import static org.firstinspires.ftc.teamcode.robot.Constants.ENABLE_AUTO_AIM;
import static org.firstinspires.ftc.teamcode.robot.Constants.HEADING;
import static org.firstinspires.ftc.teamcode.robot.Constants.IS_FULL;
import static org.firstinspires.ftc.teamcode.robot.Constants.IS_IN_CLOSE;
import static org.firstinspires.ftc.teamcode.robot.Constants.IS_IN_FAR;
import static org.firstinspires.ftc.teamcode.robot.Constants.OFFSET_TURRET;
import static org.firstinspires.ftc.teamcode.robot.Constants.ROBOT_RADIUS;
import static org.firstinspires.ftc.teamcode.robot.Constants.ROBOT_X;
import static org.firstinspires.ftc.teamcode.robot.Constants.ROBOT_Y;
import static org.firstinspires.ftc.teamcode.robot.Constants.SHOOTER_RPM;
import static org.firstinspires.ftc.teamcode.robot.Constants.SHOOTER_RPM_OFFSET;
import static org.firstinspires.ftc.teamcode.robot.Constants.START_HEADING;
import static org.firstinspires.ftc.teamcode.robot.Constants.NOW;
import static org.firstinspires.ftc.teamcode.robot.Constants.CAN_SHOOT;
import static org.firstinspires.ftc.teamcode.robot.Constants.START_POSE;
import static org.firstinspires.ftc.teamcode.robot.Constants.bluePoseToHuman1;
import static org.firstinspires.ftc.teamcode.robot.Constants.bluePoseToHuman2;
import static org.firstinspires.ftc.teamcode.robot.Constants.parkblue;
import static org.firstinspires.ftc.teamcode.subsystem.Intake.stopperState;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.subsystem.Intake;

import java.util.function.Supplier;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="TELEOPREDSOLO")
public class TeleOpRedSolo extends CommandOpMode {

    private Robot robot = Robot.getInstance();
    private GamepadEx driver;
    private GamepadEx operator;
    private ElapsedTime timer;
    PathChain goToHuman;
    PathChain goToPark;

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
        robot.follower.setHeading(HEADING);
        START_HEADING = Math.toDegrees(robot.follower.getHeading());

        goToHuman = robot.follower.pathBuilder()
                .addPath(new BezierLine(robot.follower.getPose() , bluePoseToHuman1))
                .setLinearHeadingInterpolation(robot.follower.getHeading() , 0)
                .addPath(new BezierLine(robot.follower.getPose() , bluePoseToHuman2))
                .setLinearHeadingInterpolation(robot.follower.getHeading() , 0)
                .build();
        goToPark = robot.follower.pathBuilder()
                .addPath(new BezierLine(robot.follower.getPose() , parkblue))
                .setLinearHeadingInterpolation(robot.follower.getHeading() , 0)
                .build();
        if(robot.colorSensor instanceof SwitchableLight){
            ((SwitchableLight)robot.colorSensor).enableLight(true);
        }
        if(robot.colorSensor2 instanceof SwitchableLight){
            ((SwitchableLight)robot.colorSensor2).enableLight(true);
        }
        register(robot.drive); ///nush ce face da trebuie

        driver.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenHeld(
                new InstantCommand(()->robot.intake.StartIntake())
        );
        driver.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenReleased(
                new InstantCommand(()->robot.intake.StopIntake())
        );
        driver.getGamepadButton(GamepadKeys.Button.A).whenPressed(
                new InstantCommand(()->robot.turret.ToggleShooter())
        );
        driver.getGamepadButton(GamepadKeys.Button.Y).whenPressed(
                new InstantCommand(()->robot.drive.ResetPositionRed())
        );
        driver.getGamepadButton(GamepadKeys.Button.B).whenHeld(
                new InstantCommand(()->stopperState = Intake.StopperState.OPEN)
        );
        driver.getGamepadButton(GamepadKeys.Button.B).whenReleased(
                new InstantCommand(()->stopperState = Intake.StopperState.CLOSE)
        );
        driver.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(
                new InstantCommand(()->OFFSET_TURRET+=5)
        );
        driver.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).whenPressed(
                new InstantCommand(()->OFFSET_TURRET-=5)
        );
        driver.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(
                new InstantCommand(()->SHOOTER_RPM_OFFSET+=100)
        );
        driver.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(
                new InstantCommand(()->SHOOTER_RPM_OFFSET-=100)
        );
        driver.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenPressed(
                new InstantCommand(()->robot.drive.ToggleDrive())
        );
        driver.getGamepadButton(GamepadKeys.Button.X).whenPressed(
                new FollowPathCommand(robot.follower , goToPark , false , 0.5)
        );
        super.run();
    }

    /// called every tick
    @Override
    public void run(){
        if(timer==null)
            timer = new ElapsedTime();
        NOW=getRuntime();
        CAN_SHOOT=true;
        robot.follower.update();
        SHOOTER_RPM=robot.turret.FlywheelSpeed(Math.sqrt(Math.pow(RED_BASKET_X - ROBOT_X, 2) + Math.pow(RED_BASKET_Y - ROBOT_Y, 2))*2.54 - ROBOT_RADIUS*2.54)+SHOOTER_RPM_OFFSET-200;
            ANGLE_POSITION = robot.turret.shooterAngle(Math.sqrt(Math.pow(BLUE_BASKET_X - ROBOT_X, 2) + Math.pow(BLUE_BASKET_Y - ROBOT_Y, 2))*2.54 - ROBOT_RADIUS*2.54)+0.3;
        robot.drive.Update(driver.getLeftY(),driver.getLeftX(),driver.getRightX());
        robot.intake.Update(); ///look in subsystem for more info
        robot.turret.Update(); ///look in subsystem for more info
        robot.vision.Update(24);
        robot.Update();
        if(ENABLE_AUTO_AIM)
            robot.turret.AutoAim(RED_BASKET_X , RED_BASKET_Y , Math.toDegrees(robot.follower.getHeading()));
        super.run();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.addData("X" , robot.follower.getPose().getX());
        telemetry.addData("Y" , robot.follower.getPose().getY());
        telemetry.addData("heading" , Math.toDegrees(robot.follower.getHeading()));
        telemetry.addData("angle" , ANGLE);
        telemetry.addData("shooterRPM" , SHOOTER_RPM);
        telemetry.addData("shooterRPM REAL" , robot.shooterUp.getVelocity());
        telemetry.addData("limelight offset" , robot.vision.Offset());
        telemetry.addData("aprilTag" , robot.vision.AprilTag());
        telemetry.addData("isInFar" , IS_IN_FAR);
        telemetry.addData("isInClose" , IS_IN_CLOSE);
        telemetry.addData("Drive Mode" , robot.drive.driveMode);
        telemetry.addData("distance" , Math.sqrt(Math.pow(RED_BASKET_X - ROBOT_X, 2) + Math.pow(RED_BASKET_Y - ROBOT_Y, 2))*2.54 + (ROBOT_RADIUS*2.54)/2);
        telemetry.addData("color1" , robot.intake.hsvValues1[0]+" "+robot.intake.hsvValues1[1]+" "+robot.intake.hsvValues1[2]);
        telemetry.addData("color2" , robot.intake.hsvValues2[0]+" "+robot.intake.hsvValues2[1]+" "+robot.intake.hsvValues2[2]);
        telemetry.update();
        timer.reset();
    }
}