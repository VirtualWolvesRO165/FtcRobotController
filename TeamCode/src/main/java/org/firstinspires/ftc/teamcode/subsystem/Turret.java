package org.firstinspires.ftc.teamcode.subsystem;

import static org.firstinspires.ftc.teamcode.robot.Constants.BLUE_BASKET_X;
import static org.firstinspires.ftc.teamcode.robot.Constants.BLUE_BASKET_Y;
import static org.firstinspires.ftc.teamcode.robot.Constants.HEADING;
import static org.firstinspires.ftc.teamcode.robot.Constants.OFFSET_TURRET;
import static org.firstinspires.ftc.teamcode.robot.Constants.ROBOT_X;
import static org.firstinspires.ftc.teamcode.robot.Constants.ROBOT_Y;
import static org.firstinspires.ftc.teamcode.robot.Constants.SHOOTER_RPM;
import static org.firstinspires.ftc.teamcode.robot.Constants.START_HEADING;
import static org.firstinspires.ftc.teamcode.robot.Constants.TURRET_CLOSE_POSITION;
import static org.firstinspires.ftc.teamcode.robot.Constants.TURRET_FAR_POSITION;

import com.acmerobotics.dashboard.config.Config;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.controller.PIDController;

import org.firstinspires.ftc.teamcode.robot.Robot;

@Config
public class Turret extends SubsystemBase {

    private final Robot robot = Robot.getInstance();
    public enum ShooterState{
        FORWARD , /// OUTTAKE
        REVERSE , /// EJECT , not implemented
        STOP ///STOP
    }

    public enum TurretRotationState{
        LEFT , /// rotate the turret left
        RIGHT , /// rotate the turret right
        STOP ///the turret stays in place
    }

    public enum AimState{
        FAR ,
        CLOSE
    }

    public enum ShooterAngleState {
        UP,
        /// rotate the turret left
        DOWN,
        /// rotate the turret right
        STOP ///the turret stays in place
    }

    public static ShooterState shooterState = ShooterState.STOP; ///initial state of shooter
    public static TurretRotationState turretRotationState = TurretRotationState.STOP; ///initial state of turret
    public static ShooterAngleState shooterAngleState = ShooterAngleState.STOP;
    public static AimState aimState = AimState.CLOSE;

    public PIDController turretPID = new PIDController(0.0035,0.00,0.00);
    public static double kp=0.0035,ki=0.00,kd=0.00 , kf=0;

    /// called every tick in teleOP
    public void Update(){
        /// checks state of shooter and supplies the needed power
        switch (shooterState){
            case FORWARD:
                robot.shooterUp.setVelocity(SHOOTER_RPM);
                robot.shooterDown.setVelocity(SHOOTER_RPM);
                break;
            case REVERSE:
                robot.shooterUp.setVelocity(-SHOOTER_RPM);
                robot.shooterDown.setVelocity(-SHOOTER_RPM);
                break;
            case STOP:
                robot.shooterUp.setVelocity(0);
                robot.shooterDown.setVelocity(0);
                break;
        }

        /// check state of turret and supplies the needed power
//        switch(turretRotationState){
//            case LEFT:
//                robot.shooterRotation.setPower(TURRET_ROTATION_POWER);
//                break;
//            case RIGHT:
//                robot.shooterRotation.setPower(-TURRET_ROTATION_POWER);
//                break;
//            case STOP:
//                robot.shooterRotation.setPower(0);
//                break;
//        }


        switch (shooterAngleState){
            case UP:
                robot.shooterAngle.setPosition(robot.shooterAngle.getPosition()+0.04);
                break;
            case DOWN:
                robot.shooterAngle.setPosition(robot.shooterAngle.getPosition()-0.04);
                break;
            case STOP:
                break;
        }
    }

    /// switches the state of shooter between STOP and FORWARD
    public void ToggleShooter(){
         if(shooterState == ShooterState.STOP)
            shooterState = ShooterState.FORWARD;
        else
            shooterState = ShooterState.STOP;
    }

    public void AdjustRPM(int d){
        if(d==1 && SHOOTER_RPM<6000)
            SHOOTER_RPM+=300;
        else
            if(d==-1 && SHOOTER_RPM>0)
                SHOOTER_RPM-=300;
    }

    public void StartShooter(){
        shooterState = ShooterState.FORWARD;
    }
    public void StopShooter(){
        shooterState = ShooterState.STOP;
    }

//    public void AutoAim(double targetX , double targetY , double heading){
//        double angle = Math.toDegrees(Math.atan2(targetY-ROBOT_Y , targetX-ROBOT_X-ROBOT_X))+OFFSET_TURRET+(START_HEADING-heading);
//        int shooterPos=robot.shooterRotation.getCurrentPosition();
//        turretPID = new PIDController(kp , ki , kd);
//        turretPID.setPID(kp , ki , kd);
//        double pid = turretPID.calculate(shooterPos,CalculateTarget(angle));
//        robot.shooterRotation.setPower(pid+kf);
//    }


    public int CalculateTarget(double angle){
        return (int)(450*(angle)/90);
    }

    public void UpdateTurret(int target){
        int shooterPos=robot.shooterRotation.getCurrentPosition();
        turretPID = new PIDController(kp , ki , kd);
        turretPID.setPID(kp , ki , kd);
        double pid = turretPID.calculate(shooterPos,target);
        robot.shooterRotation.setPower(pid+kf);
    }

    public void ToggleAim(){
        if(aimState==AimState.FAR)
            aimState = AimState.CLOSE;
        else
            aimState=AimState.FAR;
    }

}
