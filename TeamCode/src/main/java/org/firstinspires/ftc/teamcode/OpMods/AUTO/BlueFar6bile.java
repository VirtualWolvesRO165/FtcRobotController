package org.firstinspires.ftc.teamcode.OpMods.AUTO;

import static org.firstinspires.ftc.teamcode.robot.Constants.BLUE_BASKET_X;
import static org.firstinspires.ftc.teamcode.robot.Constants.BLUE_BASKET_Y;
import static org.firstinspires.ftc.teamcode.robot.Constants.OFFSET_TURRET;
import static org.firstinspires.ftc.teamcode.robot.Constants.ROBOT_X;
import static org.firstinspires.ftc.teamcode.robot.Constants.ROBOT_Y;
import static org.firstinspires.ftc.teamcode.robot.Constants.HEADING;
import static org.firstinspires.ftc.teamcode.robot.Constants.SHOOTER_RPM_OFFSET;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.robot.Robot;

@Config
@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="BlueFar6Bile")
public class BlueFar6bile extends OpMode {

    private Robot robot = Robot.getInstance();
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;
    private double startHeading;

    public Pose startPose = new Pose(56, 8);

    public PathChain Path1;
    public PathChain Path2;

    public void buildPaths() {
        Path1 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(56.000, 8.000),

                                new Pose(56.000, 16.000)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(0))

                .build();
        Path2 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(56.000, 16.000),

                                new Pose(35.000, 13.000)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(0))

                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                        follower.followPath(Path2 , true);
                        setPathState(-1);
                break;
        }
    }

    /**
     * These change the states of the paths and actions. It will also reset the timers of the individual switches
     **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    /**
     * This is the main loop of the OpMode, it will run repeatedly after clicking "Play".
     **/
    public void loop() {

        // These loop the movements of the robot, these must be called continuously in order to work
        follower.update();
        autonomousPathUpdate();
        robot.intake.Update();
        robot.turret.AutoAim(BLUE_BASKET_X , BLUE_BASKET_Y , Math.toDegrees(follower.getHeading()));
        robot.shooterAngle.setPosition(robot.turret.shooterAngle(Math.sqrt(Math.pow(BLUE_BASKET_X - ROBOT_X, 2) + Math.pow(BLUE_BASKET_Y - ROBOT_Y, 2))));
        robot.shooterUp.setVelocity(robot.turret.FlywheelSpeed(Math.sqrt(Math.pow(BLUE_BASKET_X - ROBOT_X, 2) + Math.pow(BLUE_BASKET_Y - ROBOT_Y, 2)))+SHOOTER_RPM_OFFSET);
        robot.shooterDown.setVelocity(robot.turret.FlywheelSpeed(Math.sqrt(Math.pow(BLUE_BASKET_X - ROBOT_X, 2) + Math.pow(BLUE_BASKET_Y - ROBOT_Y, 2)))+SHOOTER_RPM_OFFSET);
        robot.vision.Update(20);
        ROBOT_X = follower.getPose().getX();
        ROBOT_Y = follower.getPose().getY();
        HEADING = Math.toDegrees(follower.getHeading());
        // Feedback to Driver Hub for debugging
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", HEADING);
        telemetry.update();
    }

    /**
     * This method is called once at the init of the OpMode.
     **/
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();
        robot.initAuto(hardwareMap);
        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);
        startHeading = Math.toDegrees(follower.getHeading());
        robot.limelight.setPollRateHz(100);
        robot.limelight.pipelineSwitch(1);
        robot.limelight.start();

    }

    /**
     * This method is called continuously after Init while waiting for "play".
     **/
    public void init_loop() {
    }

    /**
     * This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system
     **/
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    /**
     * We do not use this because everything should automatically disable
     **/
    public void stop() {
    }
}