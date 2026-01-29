package org.firstinspires.ftc.teamcode.OpMods.AUTO;

import static org.firstinspires.ftc.teamcode.robot.Constants.ANGLE_AUTO_POSITION;
import static org.firstinspires.ftc.teamcode.robot.Constants.INTAKE_POWER;
import static org.firstinspires.ftc.teamcode.robot.Constants.TURRET_TARGET;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;


import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.subsystem.Turret;

@Config
@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="BlueFar12Bile")
public class BlueFar12Bile extends OpMode {

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
    public PathChain Path8;
    public PathChain Path9;
    public PathChain Path10;
    public PathChain Path11;

    public void buildPaths() {
        Path1 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(56.000, 8.000),

                                new Pose(44.000, 36.000)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))

                .build();

        Path2 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(44.000, 36.000),

                                new Pose(21.000, 36.000)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))

                .build();

        Path3 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(21.000, 36.000),

                                new Pose(58.000, 17.000)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))

                .build();

        Path4 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(58.000, 17.000),

                                new Pose(44.000, 60.000)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))

                .build();

        Path5 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(44.000, 60.000),

                                new Pose(27.000, 60.000)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))

                .build();

        Path6 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(27.000, 60.000),

                                new Pose(21.000, 70.000)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))

                .build();

        Path7 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(21.000, 70.000),

                                new Pose(58.000, 17.000)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))

                .build();

        Path8 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(58.000, 17.000),

                                new Pose(27, 9)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))

                .build();

        Path9 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(27, 9),

                                new Pose(15, 10)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))

                .build();

        Path10 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(15, 10),

                                new Pose(58.000, 17.000)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))

                .build();

        Path11 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(58.000, 17.000),

                                new Pose(35.000, 8.000)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))

                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                TURRET_TARGET=500;
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
                    TURRET_TARGET=-300;
                    robot.intake.StartIntake();
                    follower.followPath(Path2 , .5 ,true);
                    setPathState(2);
                }
                break;
            case 2:
                if (!follower.isBusy()) {
                    robot.turret.StartShooter();
                    follower.followPath(Path3 , true);
                    setPathState(3);
                }
                break;
            case 3:
                if (!follower.isBusy()) {

                    robot.intake.OpenStopper();
                    if(pathTimer.getElapsedTimeSeconds()>5){
                        robot.intake.CloseStopper();
                        robot.intake.StopIntake();

                        follower.followPath(Path4 , true);
                        setPathState(4);
                    }
                }
                break;
            case 4:
                if (!follower.isBusy()) {
                    robot.intake.StartIntake();
                    follower.followPath(Path5 , .5, true);
                    setPathState(5);
                }
                break;
            case 5:
                if (!follower.isBusy()) {
                    robot.intake.StopIntake();
                        follower.followPath(Path6 , true);
                        setPathState(6);
                    }
                break;
            case 6:
                if(!follower.isBusy()){
                        robot.turret.StartShooter();
                        follower.followPath(Path7 , true);
                        setPathState(7);
                }
                break;
            case 7:
                if(!follower.isBusy()) {

                    if(pathTimer.getElapsedTimeSeconds()>2) {
                        robot.intake.OpenStopper();
                        robot.intake.StartIntake();
                        if (pathTimer.getElapsedTimeSeconds() > 5) {
                            robot.intake.CloseStopper();
                            robot.intake.StopIntake();

                            follower.followPath(Path8, true);
                            setPathState(8);
                        }
                    }
                }
                break;
            case 8:
                if(!follower.isBusy()){
                    robot.intake.StartIntake();
                    follower.followPath(Path9 , .5 ,true);
                }
                break;
                case 9:
                    if(!follower.isBusy()){
                        robot.turret.StartShooter();
                        follower.followPath(Path10 , true);
                        setPathState(10);
                    }
                    break;

            case 10:
                robot.intake.OpenStopper();
                if(pathTimer.getElapsedTimeSeconds()>5){
                    robot.intake.CloseStopper();
                    robot.intake.StopIntake();

                    follower.followPath(Path11 , true);
                    setPathState(11);
                }
            break;
            case 11:
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