package org.firstinspires.ftc.teamcode.robot;

import com.acmerobotics.dashboard.config.Config;

@Config
public class Constants {
    public enum OpModTypes{
        TELEOP ,
        AUTO
    }

    public enum Alliance {
        BLUE,
        RED
    }
    public static OpModTypes opModType;
    public static Alliance alliance;

    /// DRIVE
    public static double ROBOT_X=56;
    public static double ROBOT_Y=8;

    ///INTAKE
    public static double INTAKE_POWER=-.7;
    public static double DROPDOWN_REST_POSITION=1;
    public static double DROPDOWN_ACTIVE_POSITION=0;
    public static double STOPPER_CLOSE_POSITION=1;
    public static double STOPPER_OPEN_POSITION=0;

    /// TURRET
    public static double SHOOTER_RPM =6000;
    public static double SHOOTER_POWER =1;
    public static int TURRET_FAR_POSITION=500;
    public static int TURRET_CLOSE_POSITION=0;
    public static double ANGLE_AUTO_POSITION=.8;
    public static double TURRET_ROTATION_POWER=0.35;
    public static final double BLUE_BASKET_X=0;
    public static final double BLUE_BASKET_Y=144;
    public static final double RED_BASKET_X=136;
    public static final double RED_BASKET_Y=136;
    public static double BASKET_X;
    public static double BASKET_Y;
    public static int OFFSET_TURRET=-30;

    public static double START_HEADING;
    public static double HEADING;

    public static int TURRET_TARGET=0;

    public static final double TICKS_PER_REV = 537.7;   // encoder intern
    public static final double GEAR_RATIO = 19.2;       // raport cutie 5203 Yellow

    public static final double MAX_TOTAL_ROT = Math.toRadians(300);

    ///UTIL
    public static double NOW;




}
