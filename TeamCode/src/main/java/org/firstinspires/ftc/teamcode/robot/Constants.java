package org.firstinspires.ftc.teamcode.robot;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.hardware.Gamepad;

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


    /// ROBOT
    public static double ROBOT_X;
    public static double ROBOT_Y;
    public static double ROBOT_RADIUS=8.5;
    public static Pose START_POSE;
    public static double START_HEADING;
    public static double HEADING;

    /// DRIVE
    public static boolean IS_IN_FAR=false;
    public static boolean IS_IN_CLOSE=false;
    public static boolean IS_BEFORE_IN_FAR=false;
    public static boolean IS_BEFORE_IN_CLOSE=false;
    public static boolean IS_IN_PARKING = false;
    public static boolean IS_FULL=false;
    public static boolean CAN_SHOOT=false;


    ///INTAKE
    public static double INTAKE_POWER=-1;
    public static double DROPDOWN_REST_POSITION=1;
    public static double DROPDOWN_ACTIVE_POSITION=0;
    public static double STOPPER_CLOSE_POSITION=1;
    public static double STOPPER_OPEN_POSITION=0;

    /// TURRET
    public static double SHOOTER_RPM =6000;
    public static double SHOOTER_RPM_OFFSET=200;
    public static double ANGLE_AUTO_POSITION=1;
    public static int OFFSET_TURRET=0;
    public static int ADDITIONAL_OFFSET_TURRET=0;
    public static double ANGLE_POSITION;
    public static int TURRET_TARGET=0;
    public static boolean ENABLE_AUTO_AIM=true;

    ///UTIL
    public static double NOW;
    public static double ANGLE;

    /// FIELD
    public static double BLUE_BASKET_X=0;
    public static double BLUE_BASKET_Y=144;
    public static double RED_BASKET_X=144;
    public static double RED_BASKET_Y=144;

    /// Controller
    public static Gamepad.RumbleEffect enteredLaunchZone = new Gamepad.RumbleEffect.Builder()
                    .addStep(1.0 , 1.0 , 300)
                    .build();
    public static Gamepad.RumbleEffect leavingLaunchZone = new Gamepad.RumbleEffect.Builder()
                    .addStep(1.0 , 1.0 , 100)
                    .build();
    public static Gamepad.RumbleEffect endGame = new Gamepad.RumbleEffect.Builder()
                    .addStep(1.0 , 1.0 , 300)
                    .addStep(0 , 0 , 100)
                    .addStep(1, 1, 300)
                    .build();
    public static Gamepad.RumbleEffect findParking = new Gamepad.RumbleEffect.Builder()
                    .addStep(1.0 , 1.0 , 300)
                    .addStep(0 , 0 , 100)
                    .addStep(1, 1, 300)
                    .addStep(0 , 0 , 100)
                    .addStep(1, 1, 300)
                    .build();

    public static Pose bluePoseToHuman1 = new Pose(111,12);
    public static Pose bluePoseToHuman2 = new Pose(132,12);

}
