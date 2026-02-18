package org.firstinspires.ftc.teamcode.subsystem;

import static org.firstinspires.ftc.teamcode.robot.Constants.ADDITIONAL_OFFSET_TURRET;
import static org.firstinspires.ftc.teamcode.robot.Constants.ANGLE;
import static org.firstinspires.ftc.teamcode.robot.Constants.ANGLE_POSITION;
import static org.firstinspires.ftc.teamcode.robot.Constants.IS_IN_CLOSE;
import static org.firstinspires.ftc.teamcode.robot.Constants.IS_IN_FAR;
import static org.firstinspires.ftc.teamcode.robot.Constants.OFFSET_TURRET;
import static org.firstinspires.ftc.teamcode.robot.Constants.ROBOT_X;
import static org.firstinspires.ftc.teamcode.robot.Constants.ROBOT_Y;
import static org.firstinspires.ftc.teamcode.robot.Constants.SHOOTER_RPM;
import static org.firstinspires.ftc.teamcode.robot.Constants.START_HEADING;
import static org.firstinspires.ftc.teamcode.robot.Constants.TURRET_ENCOUDER_TICKS;
import static org.firstinspires.ftc.teamcode.robot.Constants.TURRET_OFFSET_X;
import static org.firstinspires.ftc.teamcode.robot.Constants.TURRET_OFFSET_Y;

import com.acmerobotics.dashboard.config.Config;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.controller.PIDController;

import org.firstinspires.ftc.teamcode.robot.Robot;

@Config
public class Turret extends SubsystemBase {

    private final Robot robot = Robot.getInstance();

    public enum ShooterState {
        FORWARD,
        REVERSE,
        STOP
    }
    public enum ShooterAngleState {
        UP,
        DOWN,
        STOP
    }

    public static ShooterState shooterState = ShooterState.STOP;
    public static ShooterAngleState shooterAngleState = ShooterAngleState.STOP;

    public PIDController turretPID = new PIDController(0.0035, 0.0, 0.0);
    public static double kp = 0.0035, ki = 0.0, kd = 0.0, kf = 0.0;



    // ================== UPDATE ==================
    public void Update() {
        // ===== SHOOTER cu VOLTAGE COMPENSATION =====
        double voltage = robot.batteryVoltage.getVoltage();
        double compensatedRPM = SHOOTER_RPM * (13.0 / voltage);
        switch (shooterState) {
            case FORWARD:
                robot.shooterUp.setVelocity(compensatedRPM);
                robot.shooterDown.setVelocity(compensatedRPM);
                break;
            case REVERSE:
                robot.shooterUp.setVelocity(-compensatedRPM);
                robot.shooterDown.setVelocity(-compensatedRPM);
                break;
            case STOP:
                robot.shooterUp.setVelocity(0);
                robot.shooterDown.setVelocity(0);
                break;
        }
        robot.shooterAngle.setPosition(ANGLE_POSITION);
    }
    public void UpdateAuto() {
        switch (shooterAngleState) {
            case UP:
                robot.shooterAngle.setPosition(
                        robot.shooterAngle.getPosition() + 0.04
                );
                break;

            case DOWN:
                robot.shooterAngle.setPosition(
                        robot.shooterAngle.getPosition() - 0.04
                );
                break;

            case STOP:
                break;
        }
    }

    // ================== SHOOTER CONTROL ==================
    public void ToggleShooter() {
        if (shooterState == ShooterState.STOP)
            shooterState = ShooterState.FORWARD;
        else
            shooterState = ShooterState.STOP;
    }


    public void StartShooter() {
        shooterState = ShooterState.FORWARD;
    }

    public void StopShooter() {
        shooterState = ShooterState.STOP;
    }
    public void AutoAim(double targetX, double targetY, double heading) {
        double angle = Math.toDegrees(Math.atan2(targetY - ROBOT_Y, targetX - ROBOT_X)) - ADDITIONAL_OFFSET_TURRET + OFFSET_TURRET +(START_HEADING-heading);
        if(angle>90 && angle<270)
            angle = 90;
        else{
            if(angle<360 && angle>270)
                angle =-(360-angle);
        }
        ANGLE=angle;
        int shooterPos = robot.shooterRotation.getCurrentPosition();
        turretPID.setPID(kp, ki, kd);
        double pid = turretPID.calculate(shooterPos, CalculateTarget(angle));

        robot.shooterRotation.setPower(pid + kf);
    }
    public int CalculateTarget(double angle) {
        return (int) (TURRET_ENCOUDER_TICKS * angle / 90);
    }

//    public void AutoAutoAim(double targetX, double targetY, double heading , int auto) {
//        double angle = Math.toDegrees(Math.atan2(targetY - ROBOT_Y, targetX - ROBOT_X)) + OFFSET_TURRET -ADDITIONAL_OFFSET_TURRET+ (START_HEADING - heading);
//        if(angle>=90 && angle<270)
//            angle = 90;
//        else{
//            if(angle<=360 && angle>=270)
//                angle =-(360-angle);
//        }
//        angle += auto;
//        int shooterPos = robot.shooterRotation.getCurrentPosition();
//        turretPID.setPID(kp, ki, kd);
//        double pid = turretPID.calculate(shooterPos, CalculateTarget(angle));
//        robot.shooterRotation.setPower(pid + kf);
//    }


    public void UpdateTurret(int target) {
        int shooterPos = robot.shooterRotation.getCurrentPosition();
        turretPID.setPID(kp, ki, kd);
        double pid = turretPID.calculate(shooterPos, target);
        robot.shooterRotation.setPower(pid + kf);
    }

    public double FlywheelSpeed(double distance){
        return  3.59448*distance+1098.96682;
    }

    public double shooterAngle(double distance){
        return 5.26938*Math.pow(10,-9)*Math.pow(distance,4)+0.00000350072*Math.pow(distance,3)-0.000833057*Math.pow(distance,2)+0.0901116*distance-3.53654;
    }

}
