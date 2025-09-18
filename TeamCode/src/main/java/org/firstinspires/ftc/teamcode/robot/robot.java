package org.firstinspires.ftc.teamcode.robot;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.Robot;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystem.Drive;

public class robot extends Robot {

    private final Telemetry telemetry;;
    private  final HardwareMap hardwareMap;
    private final robotHardware robotHardware;
    public Drive drive;
    public static enum OpModeType {TELEOP , AUTO}
    public static OpModeType opModeType = OpModeType.TELEOP;
    public robot(Telemetry telemetry , HardwareMap hardwareMap , OpModeType opModeType){
        this.telemetry = new MultipleTelemetry(telemetry , FtcDashboard.getInstance().getTelemetry());
        this.hardwareMap = hardwareMap;
        this.opModeType = opModeType;

        robotHardware = new robotHardware(
                telemetry ,
                hardwareMap.getAll(LynxModule.class),
                hardwareMap.voltageSensor
        );

        init();

    }

    public void init(){
        drive = new Drive(
                hardwareMap.get(DcMotorEx.class , "leftFront"),
                hardwareMap.get(DcMotorEx.class , "rightFront"),
                hardwareMap.get(DcMotorEx.class , "leftBack"),
                hardwareMap.get(DcMotorEx.class , "rightBack")
        );
    }

    public void read(){
        robotHardware.read();
        drive.read();
    }

    public void loop(){
        robotHardware.loop();
        drive.loop();
    }

    public void write(){
        robotHardware.write();
        drive.write();
        telemetry.update();

    }

}
