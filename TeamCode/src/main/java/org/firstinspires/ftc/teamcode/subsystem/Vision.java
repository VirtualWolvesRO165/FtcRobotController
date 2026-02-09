package org.firstinspires.ftc.teamcode.subsystem;

import static org.firstinspires.ftc.teamcode.robot.Constants.ADDITIONAL_OFFSET_TURRET;

import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.robot.Robot;

public class Vision extends SubsystemBase {

    private final Robot robot = Robot.getInstance();
    public double[] pythonOutputs;

    public enum LimelightState {
        APRIL_TAG
    }

    public static LimelightState limelightState = LimelightState.APRIL_TAG;

    public void Update(int id) {
        switch (limelightState) {
            case APRIL_TAG:
                GoalOffsetCalculator(id);
        }
    }

    public double Offset() {
        if (pythonOutputs != null)
            return pythonOutputs[1];
        return 69;
    }

    public double AprilTag(){
        if(pythonOutputs!=null)
            return pythonOutputs[2];
        return 69;
    }
    public void GoalOffsetCalculator(int id) {
        pythonOutputs = robot.limelight.getLatestResult().getPythonOutput();
        if (pythonOutputs != null && pythonOutputs.length > 0) {
            if (pythonOutputs[2] == id) {
                ADDITIONAL_OFFSET_TURRET = (int) pythonOutputs[1];
            } else
                ADDITIONAL_OFFSET_TURRET = 0;
        }
    }
}
