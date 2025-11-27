package org.firstinspires.ftc.teamcode.subsystem;


import com.acmerobotics.dashboard.config.Config;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.robot.Robot;

@Config
public class Intake extends SubsystemBase {
        private final Robot robot = Robot.getInstance();
        private boolean power=false;
        public static double intakePower=0.7;

        public void PowerIntake(){
            power=!power;
            if(power)
                robot.intakeMotor.setPower(intakePower);
            else
                robot.intakeMotor.setPower(0);
        }
}
