package org.firstinspires.ftc.teamcode.subsystem;


import static org.firstinspires.ftc.teamcode.robot.Constants.CAN_SHOOT;
import static org.firstinspires.ftc.teamcode.robot.Constants.DROPDOWN_ACTIVE_POSITION;
import static org.firstinspires.ftc.teamcode.robot.Constants.DROPDOWN_REST_POSITION;
import static org.firstinspires.ftc.teamcode.robot.Constants.INTAKE_POWER;
import static org.firstinspires.ftc.teamcode.robot.Constants.IS_FULL;
import static org.firstinspires.ftc.teamcode.robot.Constants.IS_IN_CLOSE;
import static org.firstinspires.ftc.teamcode.robot.Constants.IS_IN_FAR;
import static org.firstinspires.ftc.teamcode.robot.Constants.SHOOTER_RPM;
import static org.firstinspires.ftc.teamcode.robot.Constants.STOPPER_CLOSE_POSITION;
import static org.firstinspires.ftc.teamcode.robot.Constants.STOPPER_OPEN_POSITION;

import android.graphics.Color;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.robot.Robot;

@Config
public class Intake extends SubsystemBase {
        private final Robot robot = Robot.getInstance();
        private ElapsedTime fullCheckTimer = new ElapsedTime();
        private boolean isFullCheck=false;
        public static NormalizedRGBA colors= new NormalizedRGBA();
      public static final float[] hsvValues = new float[3];


    public enum IntakeState{
            FORWORD , ///INTAKE
            REVERSE , ///EJECT, not implemented
            STOP ///STOP
        }

        public enum DropDownState{
            RESTING , ACTIVE ///not implemented
        }

        public enum StopperState{
            OPEN , CLOSE
        }

        public static IntakeState intakeState = IntakeState.STOP; ///INITIAL STATE OF INTAKE
        public static DropDownState dropDownState = DropDownState.RESTING; ///INITIAL STATE OF DROPDOWN , not implemented
        public static StopperState stopperState = StopperState.CLOSE;

        /// called every tick in teleOP
        public void Update() {
            /// checks the state of intake and supplies the needed power
            switch (intakeState){
                case FORWORD:
                    robot.intakeMotor.setPower(INTAKE_POWER);

                    break;
                case REVERSE:
                    robot.intakeMotor.setPower(-INTAKE_POWER);

                    break;
                case STOP:
                    robot.intakeMotor.setPower(0);

                    break;
            }
                /// not implemented
            switch (dropDownState){
                case ACTIVE:
                    robot.dropDownServoLeft.setPosition(DROPDOWN_ACTIVE_POSITION);
                    robot.dropDownServoRight.setPosition(DROPDOWN_ACTIVE_POSITION);
                    break;
                case RESTING:
                    robot.dropDownServoLeft.setPosition(DROPDOWN_REST_POSITION);
                    robot.dropDownServoRight.setPosition(DROPDOWN_REST_POSITION);
                    break;
            }

            switch (stopperState){
                case OPEN:
                    if(CAN_SHOOT)
                        robot.stopper.setPosition(STOPPER_OPEN_POSITION);
                    break;
                case CLOSE:
                        robot.stopper.setPosition(STOPPER_CLOSE_POSITION);
            }


        }

        /// switches the state of intake between STOP and FORWARD
        public void ToggleIntake(){
            if(intakeState == IntakeState.STOP)
                intakeState = IntakeState.FORWORD;
            else
                intakeState = IntakeState.STOP;
        }

        public void StartIntake(){
            intakeState = IntakeState.FORWORD;
        }

        public void StopIntake(){
            intakeState = IntakeState.STOP;
        }

        public void OpenStopper(){
            stopperState=StopperState.OPEN;
        }
        public void CloseStopper(){
            stopperState=StopperState.CLOSE;
        }

        public void CheckIntake() {
            if(robot.distanceSensor.getDistance(DistanceUnit.CM)<14 && robot.distanceSensor2.getDistance(DistanceUnit.CM)<7){
                IS_FULL=true;
            }
            if(robot.distanceSensor.getDistance(DistanceUnit.CM)>14 && robot.distanceSensor2.getDistance(DistanceUnit.CM)>7){
                IS_FULL=false;
            }
        }
}

