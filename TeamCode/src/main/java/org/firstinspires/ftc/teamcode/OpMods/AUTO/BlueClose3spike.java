package org.firstinspires.ftc.teamcode.OpMods.AUTO;

import static org.firstinspires.ftc.teamcode.robot.Constants.ADDITIONAL_OFFSET_TURRET;
import static org.firstinspires.ftc.teamcode.robot.Constants.ANGLE;
import static org.firstinspires.ftc.teamcode.robot.Constants.ANGLE_POSITION;
import static org.firstinspires.ftc.teamcode.robot.Constants.BLUE_BASKET_X;
import static org.firstinspires.ftc.teamcode.robot.Constants.BLUE_BASKET_Y;
import static org.firstinspires.ftc.teamcode.robot.Constants.CAN_SHOOT;
import static org.firstinspires.ftc.teamcode.robot.Constants.ROBOT_RADIUS;
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
@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="BlueClose3spike")
public class BlueClose3spike extends OpMode {

    private Robot robot = Robot.getInstance();
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;
    private double startHeading;

    public Pose startPose = new Pose(24.8, 124.8);

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
                                new Pose(26.800, 127.000),

                                new Pose(60.000, 84.000)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))

                .build();

        Path2 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(60.000, 84.000),
                                new Pose(57.360, 53.011),
                                new Pose(17.000, 60.000)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))

                .build();

        Path3 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(17.000, 60.000),

                                new Pose(60.000, 84.000)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))

                .build();

        Path4 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(60.000, 84.000),
                                new Pose(40.272, 66.866),
                                new Pose(21.100, 65.300)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))

                .build();

        Path5 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(21.100, 65.300),
                                new Pose(21.952, 56.000),
                                new Pose(17.903, 55.210),
                                new Pose(12.387, 50.781)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(145))

                .build();

        Path6 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(12.387, 50.781),

                                new Pose(60.000, 84.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(145), Math.toRadians(180))

                .build();

        Path7 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(60.000, 84.000),
                                new Pose(56.921, 60.948),
                                new Pose(55.144, 29.905),
                                new Pose(18.000, 35.000)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))

                .build();

        Path8 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(18.000, 35.000),

                                new Pose(60.000, 84.000)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))

                .build();

        Path9 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(60.000, 84.000),

                                new Pose(22.000, 84.000)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))

                .build();

        Path10 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(22.000, 84.000),

                                new Pose(60.000, 84.000)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))

                .build();

        Path11 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(60.000, 84.000),

                                new Pose(55.800, 112.800)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(0))

                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                robot.turret.StartShooter();
                robot.intake.CloseStopper();
                follower.followPath(Path1 , true);
                setPathState(1);
                break;

            case 1:
                if(!follower.isBusy()){
                    robot.intake.StartIntake();
                    if(pathTimer.getElapsedTimeSeconds()>3.7)
                    {
                        robot.intake.OpenStopper();
                        if(pathTimer.getElapsedTimeSeconds()>4.3){
                            robot.intake.CloseStopper();
                            setPathState(2);
                        }
                    }
                }
                break;
            case 2:
                if (!follower.isBusy()) {
                    robot.intake.StartIntake();
                    follower.followPath(Path2 ,.7 ,  true);
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
                    if(pathTimer.getElapsedTimeSeconds()>3.4){
                        robot.intake.StartIntake();
                        robot.intake.OpenStopper();
                        if (pathTimer.getElapsedTimeSeconds() > 4) {
                            robot.intake.CloseStopper();
                            follower.followPath(Path4, 1, true);
                            setPathState(5);
                        }
                    }
                }
                break;
            case 5:
                if (!follower.isBusy()) {
                    robot.intake.CloseStopper();
                    if(pathTimer.getElapsedTimeSeconds()>1.9){
                        follower.followPath(Path5, 1, true);
                        setPathState(6);
                    }
                }
                break;
            case 6:
                if(!follower.isBusy()) {
                    if (pathTimer.getElapsedTimeSeconds() > 2) {
                        robot.intake.CloseStopper();
                        follower.followPath(Path6, true);
                        setPathState(7);
                    }
                }
                break;
            case 7:
                if(!follower.isBusy()){
                    if(pathTimer.getElapsedTimeSeconds()>3.5) {
                        robot.intake.StartIntake();
                        robot.intake.OpenStopper();
                        if (pathTimer.getElapsedTimeSeconds() > 4) {
                            robot.intake.CloseStopper();
                            follower.followPath(Path7, 1, true);
                            setPathState(8);
                        }
                    }
                }
                break;
            case 8:
                if (!follower.isBusy()) {
                    robot.intake.CloseStopper();
                        follower.followPath(Path8, 1, true);
                        setPathState(9);
                }
                break;
            case 9:
                if(!follower.isBusy()){
                    if(pathTimer.getElapsedTimeSeconds()>3.5) {
                        robot.intake.StartIntake();
                        robot.intake.OpenStopper();
                        if (pathTimer.getElapsedTimeSeconds() > 4) {
                            robot.intake.CloseStopper();
                            follower.followPath(Path9, 1, true);
                            setPathState(10);
                        }
                    }
                }
                break;
            case 10:
                if (!follower.isBusy()) {
                    robot.intake.CloseStopper();
                        follower.followPath(Path10, 1, true);
                        setPathState(11);
                }
                break;
            case 11:
                if(!follower.isBusy()) {
                    if(pathTimer.getElapsedTimeSeconds()>2.2) {

                        robot.intake.OpenStopper();
                        if (pathTimer.getElapsedTimeSeconds() > 3) {
                            robot.intake.CloseStopper();
                            follower.followPath(Path11, true);
                            setPathState(12);
                        }
                    }
                }
                break;
            case 12:
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
        robot.turret.AutoAim(BLUE_BASKET_X , BLUE_BASKET_Y , Math.toDegrees(follower.getHeading()));
        SHOOTER_RPM=1470 * (13.00/robot.batteryVoltage.getVoltage());
        robot.shooterUp.setVelocity(SHOOTER_RPM);
        robot.shooterDown.setVelocity(SHOOTER_RPM);
        ANGLE_POSITION = 0.65;
        robot.shooterAngle.setPosition(ANGLE_POSITION);
        robot.vision.Update(24);
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
        ROBOT_X=55.5;
        ROBOT_Y=112.5;
        HEADING=0;
        START_HEADING=0;
    }
}