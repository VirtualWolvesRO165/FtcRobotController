package org.firstinspires.ftc.teamcode.subsystem;

import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.robot.Constants;
import org.firstinspires.ftc.teamcode.robot.Robot;

import static org.firstinspires.ftc.teamcode.robot.Constants.ROBOT_X;
import static org.firstinspires.ftc.teamcode.robot.Constants.ROBOT_Y;

public class Drive extends SubsystemBase
{
    private final Robot robot = Robot.getInstance();
    public void PowerMotor(double p_drive , double strafe , double turn){

        robot.leftFront.setPower((p_drive+strafe+turn));
        robot.leftBack.setPower((p_drive-strafe+turn));
        robot.rightFront.setPower((p_drive-strafe-turn));
        robot.rightBack.setPower((p_drive+strafe-turn));

        robot.pinpoint.update();
        ROBOT_X = robot.pinpoint.getPosX(DistanceUnit.INCH);
        ROBOT_Y = robot.pinpoint.getPosY(DistanceUnit.INCH);
    }
}
