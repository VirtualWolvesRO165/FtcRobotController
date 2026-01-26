package org.firstinspires.ftc.teamcode.OpMods.AUTO;

import static org.firstinspires.ftc.teamcode.robot.Constants.ANGLE_AUTO_POSITION;
import static org.firstinspires.ftc.teamcode.robot.Constants.INTAKE_POWER;
import static org.firstinspires.ftc.teamcode.robot.Constants.SHOOTER_RPM;
import static org.firstinspires.ftc.teamcode.robot.Constants.STOPPER_CLOSE_POSITION;
import static org.firstinspires.ftc.teamcode.robot.Constants.STOPPER_OPEN_POSITION;

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
@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="RedFar")
public class RedFar extends OpMode {

    private Robot robot = Robot.getInstance();
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;
    public Pose startPose = new Pose(88, 8);

    public PathChain Path1;

    public void buildPaths() {
        Path1 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(88.000, 8.000),

                                new Pose(104.000, 15.000)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(0))

                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                robot.shooterUp.setPower(SHOOTER_RPM);
                robot.shooterDown.setPower(SHOOTER_RPM);
                robot.shooterAngle.setPosition(ANGLE_AUTO_POSITION);
                setPathState(1);
                break;
            case 1:
                if (pathTimer.getElapsedTimeSeconds() > 2) {
                    robot.intakeMotor.setPower(INTAKE_POWER);
                    robot.stopper.setPosition(STOPPER_OPEN_POSITION);
                    setPathState(2);
                }
                break;
            case 2:
                if (pathTimer.getElapsedTimeSeconds() > 2) {
                    robot.intakeMotor.setPower(0);
                    robot.shooterUp.setPower(0);
                    robot.shooterDown.setPower(0);
                    robot.stopper.setPosition(STOPPER_CLOSE_POSITION);
                    follower.followPath(Path1 , true);
                    setPathState(3);
                }
                break;
            case 3:
                if (!follower.isBusy()) {
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
        follower.update();
        autonomousPathUpdate();
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