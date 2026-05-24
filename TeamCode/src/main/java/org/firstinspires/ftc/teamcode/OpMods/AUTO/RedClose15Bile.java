package org.firstinspires.ftc.teamcode.OpMods.AUTO;

import static org.firstinspires.ftc.teamcode.robot.Constants.ADDITIONAL_OFFSET_TURRET;
import static org.firstinspires.ftc.teamcode.robot.Constants.ANGLE;
import static org.firstinspires.ftc.teamcode.robot.Constants.ANGLE_POSITION;
import static org.firstinspires.ftc.teamcode.robot.Constants.BLUE_BASKET_X;
import static org.firstinspires.ftc.teamcode.robot.Constants.BLUE_BASKET_Y;
import static org.firstinspires.ftc.teamcode.robot.Constants.CAN_SHOOT;
import static org.firstinspires.ftc.teamcode.robot.Constants.RED_BASKET_X;
import static org.firstinspires.ftc.teamcode.robot.Constants.RED_BASKET_Y;
import static org.firstinspires.ftc.teamcode.robot.Constants.ROBOT_X;
import static org.firstinspires.ftc.teamcode.robot.Constants.ROBOT_Y;
import static org.firstinspires.ftc.teamcode.robot.Constants.HEADING;
import static org.firstinspires.ftc.teamcode.robot.Constants.SHOOTER_RPM;
import static org.firstinspires.ftc.teamcode.robot.Constants.SHOOTER_RPM_OFFSET;
import static org.firstinspires.ftc.teamcode.robot.Constants.START_HEADING;
import static org.firstinspires.ftc.teamcode.robot.Constants.TURRET_POSE_BLUE;

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

@Config
@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="RedClose15Bile")
public class RedClose15Bile extends OpMode {

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
    public PathChain Path8;
    public PathChain Path9;

    public void buildPaths() {
        Path1 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(119.4, 124.8),

                                new Pose(80.000, 84.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0),Math.toRadians(45))

                .build();

        Path2 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(80.000, 84.000),
                                new Pose(89.002, 62.539),
                                new Pose(100.023, 54.530),
                                new Pose(126.000, 60.000)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(0))

                .build();

        Path3 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(123.000, 60.000),

                                new Pose(80.000, 84.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0),Math.toRadians(45))

                .build();

        Path4 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(80.000, 84.000),
                                new Pose(95.499, 65.186),
                                new Pose(127.900, 65.300)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(0))

                .build();


        Path5 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(127.900, 65.300),
                                new Pose(119.377, 55.979),
                                new Pose(134.000, 58.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0) , Math.toRadians(25))

                .build();

        Path6 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(134.000, 58.000),

                                new Pose(80.000, 84.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(25),Math.toRadians(45))

                .build();

        Path7 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(80.000, 84.000),

                                new Pose(112.000, 89.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(0))

                .build();

        Path8 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(80.000, 84.000),

                                new Pose(126.000, 84.000)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(0))

                .build();

        Path9 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(126.000, 84.000),

                                new Pose(80.000, 84.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0),Math.toRadians(45))

                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                robot.turret.StartShooter();
                ANGLE_POSITION=1;
                robot.intake.CloseStopper();
                follower.followPath(Path1 , true);
                setPathState(1);
                break;

            case 1:
                if(!follower.isBusy()){
                    robot.intake.StartIntake();
                    if(pathTimer.getElapsedTimeSeconds()>2.5)
                    {
                        robot.intake.OpenStopper();
                        if(pathTimer.getElapsedTimeSeconds()>3.5){
                            robot.intake.CloseStopper();
                            setPathState(2);
                        }
                    }
                }
                break;
            case 2:
                if (!follower.isBusy()) {
                    ANGLE_POSITION=0.8;
                    robot.intake.StartIntake();
                    follower.followPath(Path2 ,.8 ,  true);
                    setPathState(3);
                }
                break;
            case 3:
                if (!follower.isBusy()) {
                    follower.followPath(Path3 , true);
                    setPathState(4);
                }
                break;
            case 4:
                if(!follower.isBusy()){
                    robot.intake.StartIntake();
                    robot.intake.OpenStopper();
                    if(pathTimer.getElapsedTimeSeconds()>2.8){
                        robot.intake.CloseStopper();
                        follower.followPath(Path4 , true);
                        setPathState(5);
                    }
                }
                break;
            case 5:
                if (!follower.isBusy()) {
                    robot.intake.CloseStopper();
                    follower.followPath(Path5,.8 , true);
                    setPathState(6);
                }
                break;
            case 6:
                if (pathTimer.getElapsedTimeSeconds()>2.8) {
                    robot.intake.CloseStopper();
                    follower.followPath(Path6 , true);
                    setPathState(7);
                }
                break;
            case 7:
                if(!follower.isBusy()){
                    robot.intake.StartIntake();
                    robot.intake.OpenStopper();
                    if(pathTimer.getElapsedTimeSeconds()>2.8){
                        robot.intake.CloseStopper();
                        follower.followPath(Path4 , true);
                        setPathState(8);
                    }
                }
                break;
            case 8:
                if (!follower.isBusy()) {
                    robot.intake.CloseStopper();
                    follower.followPath(Path5,.8 , true);
                    setPathState(9);
                }
                break;
            case 9:
                if (pathTimer.getElapsedTimeSeconds()>2.2) {
                    robot.intake.CloseStopper();
                    follower.followPath(Path6 , true);
                    setPathState(10);
                }
                break;
            case 10:
                if (!follower.isBusy()) {
                    robot.intake.OpenStopper();
                    if(pathTimer.getElapsedTimeSeconds()>2.8){
                        robot.intake.CloseStopper();
                        follower.followPath(Path8 , true);
                        setPathState(11);
                    }
                }
                break;
            case 11:
                if(!follower.isBusy())
                {
                    follower.followPath(Path9);
                    setPathState(12);
                }
                break;

            case 12:
                if (!follower.isBusy()) {
                    robot.intake.OpenStopper();
                    if(pathTimer.getElapsedTimeSeconds()>2.8){
                        robot.intake.CloseStopper();
                        follower.followPath(Path7 , true);
                        setPathState(13);
                    }
                }
                break;
            case 13:
                if(!follower.isBusy()){
                    robot.intake.StopIntake();
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
        SHOOTER_RPM=1420;
        robot.vision.Update(20);
        ROBOT_X = follower.getPose().getX();
        ROBOT_Y = follower.getPose().getY();
        HEADING = Math.toDegrees(follower.getHeading());
        // Feedback to Driver Hub for debugging
        telemetry.addData("path state", pathState);
        telemetry.addData("pathTimer", pathTimer.getElapsedTimeSeconds());
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
        ROBOT_X=112;
        ROBOT_Y=89;
        HEADING=0;
        START_HEADING=0;
    }
}