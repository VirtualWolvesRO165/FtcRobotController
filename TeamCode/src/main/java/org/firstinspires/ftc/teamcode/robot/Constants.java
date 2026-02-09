package org.firstinspires.ftc.teamcode.robot;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;

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
    public static double ROBOT_X;
    public static double ROBOT_Y;
    public static double ROBOT_RADIUS=25;
    public static double ENTRY_MARGIN = 2.0; // inches, tune this

    public static Pose START_POSE;
    public static boolean IS_IN_FAR=false;
    public static boolean IS_IN_CLOSE=false;

    ///INTAKE
    public static double INTAKE_POWER=-1;
    public static double DROPDOWN_REST_POSITION=1;
    public static double DROPDOWN_ACTIVE_POSITION=0;
    public static double STOPPER_CLOSE_POSITION=1;
    public static double STOPPER_OPEN_POSITION=0;

    /// TURRET
    public static double SHOOTER_RPM =6000;
    public static double SHOOTER_RPM_OFFSET=-150;
    public static double SHOOTER_RPM_AUTO=2000;
    public static double ANGLE_AUTO_POSITION=1;
    public static final double BLUE_BASKET_X=0;
    public static final double BLUE_BASKET_Y=144;
    public static final double RED_BASKET_X=144;
    public static final double RED_BASKET_Y=144;
    public static int OFFSET_TURRET=0;
    public static int ADDITIONAL_OFFSET_TURRET=0;
    public static double ANGLE_POSITION;

    public static double START_HEADING;
    public static double HEADING;

    public static int TURRET_TARGET=0;

    ///UTIL
    public static double NOW;




}
