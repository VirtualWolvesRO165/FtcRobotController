package org.firstinspires.ftc.teamcode.subsystem;

import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.robot.Robot;

public class Vision extends SubsystemBase {

    private final Robot robot = Robot.getInstance();

    public enum LimelightState{
        APRIL_TAG
    }

    public static LimelightState limelightState = LimelightState.APRIL_TAG;

}
