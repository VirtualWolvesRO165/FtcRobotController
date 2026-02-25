package org.firstinspires.ftc.teamcode.OpMods.AUTO;

import static org.firstinspires.ftc.teamcode.robot.Constants.ANGLE_AUTO_POSITION;
import static org.firstinspires.ftc.teamcode.robot.Constants.INTAKE_POWER;
import static org.firstinspires.ftc.teamcode.robot.Constants.TURRET_TARGET;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;


import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.subsystem.Turret;

@Config
@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="RedFar12Bile")
public class RedFar12Bile extends OpMode {

    private Robot robot = Robot.getInstance();
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;

    public Pose startPose = new Pose(56, 8);

    public PathChain Path1;
    public PathChain Path2;
    public PathChain Path3;
    public PathChain Path4;
    public PathChain Path5;
    public PathChain Path6;
    public PathChain Path7;

    public void buildPaths() {
        Path1 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(88.000, 7.000),
                                new Pose(84.856, 42.092),
                                new Pose(102.000, 35.000),
                                new Pose(124.000, 36.000)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(0))

                .build();

        Path2 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(124.000, 36.000),

                                new Pose(84.000, 15.000)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(0))

                .build();

        Path3 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(84.000, 15.000),
                                new Pose(103.879, 7.389),
                                new Pose(132.204, 8.235)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(0))

                .build();

        Path4 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(132.204, 8.235),

                                new Pose(84.000, 15.000)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(0))

                .build();

        Path5 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(84.000, 15.000),

                                new Pose(134.000, 16.000)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(0))

                .build();

        Path6 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(134.000, 16.000),

                                new Pose(84.000, 15.000)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(0))

                .build();

        Path7 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(84.000, 15.000),

                                new Pose(109.000, 8.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(180))

                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                robot.intake.StartIntake();
                robot.turret.StartShooter();
                if(pathTimer.getElapsedTimeSeconds()>4)
                {
                    robot.intake.OpenStopper();
                    if(pathTimer.getElapsedTimeSeconds()>6){
                        robot.intake.CloseStopper();
                        robot.intake.StopIntake();
                        follower.followPath(Path1 , true);
                        setPathState(1);
                    }
                }
                break;
            case 1:
                if (!follower.isBusy()) {
                    follower.followPath(Path2 ,true);
                    setPathState(2);
                }
                break;
            case 2:
                if(!follower.isBusy()){
                    robot.intake.StartIntake();
                    robot.intake.OpenStopper();
                    if(pathTimer.getElapsedTimeSeconds()>4)
                    {
                        robot.intake.CloseStopper();
                        follower.followPath(Path3 , true);
                        setPathState(3);
                    }
                }
                break;
            case 3:
                if(!follower.isBusy()){
                    follower.followPath(Path4,true);
                }
                break;
            case 4:
                if(!follower.isBusy()){
                    robot.intake.StartIntake();
                    robot.intake.OpenStopper();
                    if(pathTimer.getElapsedTimeSeconds()>4)
                    {
                        robot.intake.CloseStopper();
                        follower.followPath(Path5 , true);
                        setPathState(5);
                    }
                }
                break;
            case 5:
                if(!follower.isBusy()){
                    follower.followPath(Path6 , true);
                    setPathState(6);
                }
                break;
            case 6:
                if(!follower.isBusy()){
                    robot.intake.StartIntake();
                    robot.intake.OpenStopper();
                    if(pathTimer.getElapsedTimeSeconds()>4)
                    {
                        robot.intake.CloseStopper();
                        follower.followPath(Path7 , true);
                        setPathState(7);
                    }
                }
                break;
            case 7:
                if(!follower.isBusy())
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
        robot.turret.Update();
        robot.turret.UpdateTurret(TURRET_TARGET);
        robot.shooterAngle.setPosition(ANGLE_AUTO_POSITION);
        // Feedback to Driver Hub for debugging
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
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
        robot.turret.StartShooter();
        robot.shooterAngle.setPosition(ANGLE_AUTO_POSITION);
        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);


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