package org.firstinspires.ftc.teamcode.subsystem;

import static org.firstinspires.ftc.teamcode.robot.Constants.SHOOTER_POWER;
import static org.firstinspires.ftc.teamcode.robot.Constants.TURRET_ROTATION_POWER;

import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.robot.Robot;

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


    public static ShooterState shooterState = ShooterState.STOP; ///initial state of shooter
    public static TurretRotationState turretRotationState = TurretRotationState.STOP; ///initial state of turret

    /// called every tick in teleOP
    public void Update(){
        /// checks state of shooter and supplies the needed power
        switch (shooterState){
            case FORWARD:
                robot.shooterUp.setPower(SHOOTER_POWER);
                robot.shooterDown.setPower(SHOOTER_POWER);
                break;
            case REVERSE:
                robot.shooterUp.setPower(-SHOOTER_POWER);
                robot.shooterDown.setPower(-SHOOTER_POWER);
                break;
            case STOP:
                robot.shooterUp.setPower(0);
                robot.shooterDown.setPower(0);
                break;
        }

        /// check state of turret and supplies the needed power
        switch(turretRotationState){
            case LEFT:
                robot.shooterRotation.setPower(-TURRET_ROTATION_POWER);
                break;
            case RIGHT:
                robot.shooterRotation.setPower(TURRET_ROTATION_POWER);
                break;
            case STOP:
                robot.shooterRotation.setPower(0);
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

}
