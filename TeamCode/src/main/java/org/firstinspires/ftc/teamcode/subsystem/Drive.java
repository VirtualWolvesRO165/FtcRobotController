package org.firstinspires.ftc.teamcode.subsystem;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.robot.Constants;
import org.firstinspires.ftc.teamcode.robot.Robot;

import static org.firstinspires.ftc.teamcode.pedroPathing.Constants.createFollower;
import static org.firstinspires.ftc.teamcode.robot.Constants.HEADING;
import static org.firstinspires.ftc.teamcode.robot.Constants.ROBOT_X;
import static org.firstinspires.ftc.teamcode.robot.Constants.ROBOT_Y;
import static org.firstinspires.ftc.teamcode.robot.Constants.START_HEADING;

import java.util.function.Supplier;

public class Drive extends SubsystemBase
{
    private final Robot robot = Robot.getInstance();
    public static boolean slowSpeed=false , slowTurn=false;
    public void init(){

    }
    public void PowerMotor(double p_drive , double strafe , double turn){
        robot.leftFront.setPower((p_drive+strafe+turn));
        robot.leftBack.setPower((p_drive-strafe+turn));
        robot.rightFront.setPower((p_drive-strafe-turn));
        robot.rightBack.setPower((p_drive+strafe-turn));
    }
}