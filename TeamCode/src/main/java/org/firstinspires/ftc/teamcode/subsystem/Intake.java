package org.firstinspires.ftc.teamcode.subsystem;


import static org.firstinspires.ftc.teamcode.robot.Constants.DROPDOWN_ACTIVE_POSITION;
import static org.firstinspires.ftc.teamcode.robot.Constants.DROPDOWN_REST_POSITION;
import static org.firstinspires.ftc.teamcode.robot.Constants.INTAKE_POWER;
import static org.firstinspires.ftc.teamcode.robot.Constants.STOPPER_CLOSE_POSITION;
import static org.firstinspires.ftc.teamcode.robot.Constants.STOPPER_OPEN_POSITION;

import com.acmerobotics.dashboard.config.Config;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.robot.Robot;

@Config
public class Intake extends SubsystemBase {
        private final Robot robot = Robot.getInstance();

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

        public void ToggleStopper(){
            if(stopperState==StopperState.OPEN)
                stopperState=StopperState.CLOSE;
            else
                stopperState=StopperState.OPEN;
        }

        public void OpenStopper(){
            stopperState=StopperState.OPEN;
        }
        public void CloseStopper(){
            stopperState=StopperState.CLOSE;
        }


}
