package org.firstinspires.ftc.teamcode.OpMods.AUTO;

import static org.firstinspires.ftc.teamcode.robot.Constants.ANGLE;
import static org.firstinspires.ftc.teamcode.robot.Constants.ANGLE_POSITION;
import static org.firstinspires.ftc.teamcode.robot.Constants.RED_BASKET_X;
import static org.firstinspires.ftc.teamcode.robot.Constants.RED_BASKET_Y;
import static org.firstinspires.ftc.teamcode.robot.Constants.CAN_SHOOT;
import static org.firstinspires.ftc.teamcode.robot.Constants.ROBOT_X;
import static org.firstinspires.ftc.teamcode.robot.Constants.ROBOT_Y;
import static org.firstinspires.ftc.teamcode.robot.Constants.HEADING;
import static org.firstinspires.ftc.teamcode.robot.Constants.SHOOTER_RPM;
import static org.firstinspires.ftc.teamcode.robot.Constants.SHOOTER_RPM_OFFSET;
import static org.firstinspires.ftc.teamcode.robot.Constants.START_HEADING;

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
@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="RedClose9Bile")
public class RedClose9Bile extends OpMode {

    private Robot robot = Robot.getInstance();
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;
    private double startHeading;

    public Pose startPose = new Pose(119.4, 124.8);

    public PathChain Path1;
    public PathChain Path2;
    public PathChain Path3;
    public PathChain Path4;
    public PathChain Path5;
    public PathChain Path6;
    public PathChain Path7;

    public void buildPaths() {
        Path1 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(119.000, 124.40),

                                new Pose(84.000, 84.000)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(0))

                .build();

        Path2 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(84.000, 84.000),

                                new Pose(127.000, 84.000)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(0))

                .build();

        Path3 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(127.000, 84.000),

                                new Pose(84.000, 84.000)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(0))

                .build();

        Path4 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(84.000, 84.000),

                                new Pose(100.000, 60.000)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(0))

                .build();

        Path5 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(100.000, 60.000),

                                new Pose(127.000, 60.000)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(0))

                .build();

        Path6 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(127.000, 60.000),

                                new Pose(84.000, 84.000)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(0))
                .build();
        Path7 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(127.000, 60.000),

                                new Pose(112.100, 88.700)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(0))
                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                robot.turret.StartShooter();
                robot.intake.StartIntake();
                robot.intake.CloseStopper();
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
                    follower.followPath(Path2 ,.6 ,  true);
                    setPathState(3);
                }
                break;
            case 3:
                if (!follower.isBusy()) {

                    robot.turret.StartShooter();
                    follower.followPath(Path3 , true);
                    setPathState(4);

                }
                break;
            case 4:
                if (!follower.isBusy()) {
                    robot.intake.OpenStopper();
                    if(pathTimer.getElapsedTimeSeconds()>4){
                        robot.intake.CloseStopper();
                        follower.followPath(Path4 , true);
                        setPathState(5);
                    }
                }
                break;
            case 5:
                if (!follower.isBusy()) {
                    robot.intake.StartIntake();
                    follower.followPath(Path5 , .6 , true);
                    setPathState(6);
                }
                break;
            case 6:
                if (!follower.isBusy()) {
                    robot.turret.StartShooter();
                    follower.followPath(Path6 , true);
                    setPathState(7);
                }
                break;
            case 7:
                if (!follower.isBusy()) {
                    robot.intake.OpenStopper();
                    if(pathTimer.getElapsedTimeSeconds()>4){
                        robot.intake.CloseStopper();

                        follower.followPath(Path7 , .6 , true);
                        setPathState(8);
                    }
                }
                break;
            case 8:
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

        // These loop the movements of the robot, these must be called continuously in order to work
        CAN_SHOOT=true;
        follower.update();
        autonomousPathUpdate();
        robot.intake.Update();
        robot.turret.Update();
        robot.turret.AutoAim(RED_BASKET_X , RED_BASKET_Y , Math.toDegrees(follower.getHeading()));
        SHOOTER_RPM=robot.turret.FlywheelSpeed(Math.sqrt(Math.pow(RED_BASKET_X - ROBOT_X, 2) + Math.pow(RED_BASKET_Y - ROBOT_Y, 2)))+SHOOTER_RPM_OFFSET;
        ANGLE_POSITION = robot.turret.shooterAngle(Math.sqrt(Math.pow(RED_BASKET_X - ROBOT_X, 2) + Math.pow(RED_BASKET_Y - ROBOT_Y, 2))*2.54);
        robot.vision.Update(20);
        ROBOT_X = follower.getPose().getX();
        ROBOT_Y = follower.getPose().getY();
        HEADING = Math.toDegrees(follower.getHeading());
        // Feedback to Driver Hub for debugging
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", HEADING);
        telemetry.addData("shooterRPM", SHOOTER_RPM);
        telemetry.addData("angle", ANGLE_POSITION);
        telemetry.addData("turret angle", ANGLE);
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
        ROBOT_X=112.1;
        ROBOT_Y=88.7;
        HEADING=0;
        START_HEADING=0;
    }
}