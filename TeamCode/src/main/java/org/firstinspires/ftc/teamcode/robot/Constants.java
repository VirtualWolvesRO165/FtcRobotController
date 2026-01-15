package org.firstinspires.ftc.teamcode.robot;

import com.acmerobotics.dashboard.config.Config;

@Config
public class Constants {
    public enum OpModTypes{
        TELEOP ,
        AUTO
    }

    public static OpModTypes opModType;

    ///INTAKE
    public static final double INTAKE_POWER=0.7;
    public static final double DROPDOWN_REST_POSITION=1;
    public static final double DROPDOWN_ACTIVE_POSITION=0;

    /// TURRET
    public static final double SHOOTER_POWER=1;
    public static int TURRET_DIRECTION=0;
    public static final double TURRET_ROTATION_POWER=0.3;


}
