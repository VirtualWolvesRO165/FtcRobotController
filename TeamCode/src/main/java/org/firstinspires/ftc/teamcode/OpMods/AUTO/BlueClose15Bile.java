package org.firstinspires.ftc.teamcode.OpMods.AUTO;

import static org.firstinspires.ftc.teamcode.robot.Constants.BLUE_BASKET_X;
import static org.firstinspires.ftc.teamcode.robot.Constants.BLUE_BASKET_Y;
import static org.firstinspires.ftc.teamcode.robot.Constants.CAN_SHOOT;
import static org.firstinspires.ftc.teamcode.robot.Constants.ROBOT_X;
import static org.firstinspires.ftc.teamcode.robot.Constants.ROBOT_Y;
import static org.firstinspires.ftc.teamcode.robot.Constants.HEADING;
import static org.firstinspires.ftc.teamcode.robot.Constants.SHOOTER_RPM_OFFSET;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.robot.Robot;

@Config
@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="BlueClose15Bile")
public class BlueClose15Bile extends OpMode {

    private Robot robot = Robot.getInstance();
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;
    private double startHeading;

    public Pose startPose = new Pose(28, 128);

    public PathChain Path1;
    public PathChain Path2;
    public PathChain Path3;
    public PathChain Path4;
    public PathChain Path5;
    public PathChain Path6;
    public PathChain Path7;
    public PathChain Path8;

    public void buildPaths() {
        Path1 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(28.000, 128.000),

                                new Pose(60.000, 84.000)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))

                .build();

        Path2 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(60.000, 84.000),

                                new Pose(44.000, 60.000)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))

                .build();

        Path3 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(44.000, 60.000),

                                new Pose(20.000, 60.000)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))

                .build();

        Path4 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(20.000, 60.000),

                                new Pose(60.000, 84.000)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))

                .build();

        Path5 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(60.000, 84.000),

                                new Pose(15.000, 56.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                .build();

        Path6 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(15.000, 56.000),

                                new Pose(14.600, 63.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(148))

                .build();
        Path8 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(14.600, 63.000),

                                new Pose(14.600, 63.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(148), Math.toRadians(148))

                .build();

        Path7 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(14.600, 63.000),

                                new Pose(60.000, 84.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(130), Math.toRadians(180))

                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                SHOOTER_RPM_OFFSET+=150;
                robot.intake.CloseStopper();
                robot.intake.StartIntake();
                follower.followPath(Path1 , true);
                setPathState(1);
                break;

            case 1:
                if(!follower.isBusy()){
                    if(pathTimer.getElapsedTimeSeconds()>3)
                    {
                        robot.intake.OpenStopper();
                        if(pathTimer.getElapsedTimeSeconds()>5){
                            robot.intake.CloseStopper();
                            setPathState(2);
                        }
                    }
                }

                break;
            case 2:
                if (!follower.isBusy()) {
                    robot.intake.StartIntake();
                    follower.followPath(Path2,  true);
                    setPathState(3);
                }
                break;
            case 3:
                if (!follower.isBusy()) {
                    follower.followPath(Path3, .7 , true);
                    setPathState(4);

                }
                break;
            case 4:
                if (!follower.isBusy()) {
                        follower.followPath(Path4 , true);
                        setPathState(5);
                    }
                break;
            case 5:
                if (!follower.isBusy()) {
                    if (pathTimer.getElapsedTimeSeconds() > 2) {
                        robot.intake.OpenStopper();
                        if (pathTimer.getElapsedTimeSeconds() > 4) {
                            follower.followPath(Path5, true);
                            robot.intake.CloseStopper();
                            setPathState(6);
                        }
                    }
                }
                break;
            case 6:
                if (!follower.isBusy()) {
                    follower.followPath(Path6, .6 , true);
                    setPathState(7);
                }
                break;
            case 7:
                if(pathTimer.getElapsedTimeSeconds()>2){
                    follower.followPath(Path8 , true);
                    setPathState(8);
                }
                break;

            case 8:
                    if(pathTimer.getElapsedTimeSeconds()>2){
                        follower.followPath(Path7 , true);
                        setPathState(9);
                    }

                break;
            case 9:
                if (!follower.isBusy()) {
                    if (pathTimer.getElapsedTimeSeconds() > 2) {
                        robot.intake.OpenStopper();
                        if (pathTimer.getElapsedTimeSeconds() > 4) {
                            follower.followPath(Path5, true);
                            robot.intake.CloseStopper();
                            setPathState(10);
                        }
                    }
                }
                break;
            case 10:
                if (!follower.isBusy()) {
                    follower.followPath(Path6, .6 , true);
                    setPathState(11);
                }
                break;
            case 11:
                if(!follower.isBusy()){
                    follower.followPath(Path8 , true);
                    setPathState(12);
                }
                break;
            case 12:
                    if(pathTimer.getElapsedTimeSeconds()>2){
                        follower.followPath(Path7 , true);
                        setPathState(13);
                    }

                break;
            case 13:
                if (!follower.isBusy()) {
                    if (pathTimer.getElapsedTimeSeconds() > 2) {
                        robot.intake.OpenStopper();
                        if (pathTimer.getElapsedTimeSeconds() > 4) {
                            follower.followPath(Path5, true);
                            setPathState(14);
                        }
                    }
                }
                break;
            case 14:
                if(!follower.isBusy()){
                    robot.turret.StopShooter();
                    setPathState(-1);
                }
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
        CAN_SHOOT=true;
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
        ROBOT_X=20;
        ROBOT_Y=91.5;
        HEADING=0;
    }
}