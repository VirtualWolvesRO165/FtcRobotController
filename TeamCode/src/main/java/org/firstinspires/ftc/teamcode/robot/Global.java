package org.firstinspires.ftc.teamcode.robot;

import com.acmerobotics.dashboard.config.Config;

@Config
public class Global {
    public enum OpModTypes{
        TELEOP ,
        AUTO
    }
    public static int currentRoom=1;
    public static int[] roomIntake = {0 ,0 ,0};
    public static int[] roomOuttake = {0, 0, 0};

}
