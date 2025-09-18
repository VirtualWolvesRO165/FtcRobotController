package org.firstinspires.ftc.teamcode.old;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;

public class DriveTrain {

    DcMotorEx leftFront , rightFront , leftBack , rightBack;
    Gamepad gamepad;

    public void Init(DcMotorEx leftF , DcMotorEx rightF , DcMotorEx leftB , DcMotorEx rightB){
        leftFront = leftF;
        rightFront = rightF;
        leftBack = leftB;
        rightBack = rightB;
    }

    public void Drive(){
        double y = -gamepad.left_stick_y; // Remember, Y stick is reversed!
        double x = gamepad.left_stick_x;
        double rx = gamepad.right_stick_x;

        if (!gamepad.left_bumper) {
            x /= 2;
            y /= 2;
        }
        if (!gamepad.right_bumper) {
            rx /= 2;
        }

        leftFront.setPower(y + x + rx);
        leftBack.setPower(y - x + rx);
        rightFront.setPower(y - x - rx);
        rightBack.setPower(y + x - rx);
    }
}
