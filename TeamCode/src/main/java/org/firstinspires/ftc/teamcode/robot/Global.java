package org.firstinspires.ftc.teamcode.robot;

import com.acmerobotics.dashboard.config.Config;

@Config
public class Global {
    public enum OpModTypes{
        TELEOP ,
        AUTO
    }
    public static int currentRoom=1;
    public static int[] roomIntake = {0 ,180 ,370};
    public static int[] roomOuttake = {265, 445, 610};
    //-1=no bila 0=green 1=purple
    public static int[] artefactsOrder={-1 ,-1 ,-1};
    //[0]=green,[1]=purple
    public static int artefacts;
    public static boolean outtaking=false;

}
