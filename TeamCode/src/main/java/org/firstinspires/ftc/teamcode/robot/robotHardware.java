package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap.DeviceMapping;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.List;

public class robotHardware extends SubsystemBase {

    private final Telemetry telemetry;
    private final List<LynxModule> hubs;
    private final DeviceMapping<VoltageSensor> voltageSensor;
    private ElapsedTime voltageTimer = new ElapsedTime();
    public static double voltage = 0.0;
    private double loopTime = 0.0;
    private double prevLoopTime = 0.0;

    public robotHardware(Telemetry telemetry , List<LynxModule> hubs , DeviceMapping<VoltageSensor> voltageSensor){

        this.telemetry = telemetry;
        this.hubs = hubs;
        this.voltageSensor = voltageSensor;

        for(LynxModule hub : hubs){
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        voltage = voltageSensor.iterator().next().getVoltage();
    }


}
