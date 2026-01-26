package org.firstinspires.ftc.teamcode.subsystem;

import static org.firstinspires.ftc.teamcode.robot.Constants.SHOOTER_RPM;
import static org.firstinspires.ftc.teamcode.robot.Constants.TURRET_ROTATION_POWER;

import com.acmerobotics.dashboard.config.Config;
import com.seattlesolvers.solverslib.command.SubsystemBase;

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
        switch(turretRotationState){
            case LEFT:
                robot.shooterRotation.setPower(TURRET_ROTATION_POWER);
                break;
            case RIGHT:
                robot.shooterRotation.setPower(-TURRET_ROTATION_POWER);
                break;
            case STOP:
                robot.shooterRotation.setPower(0);
                break;
        }

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




}
