package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.robotcore.hardware.DcMotorEx;

public class Drive extends subsystemW{

    private final DcMotorEx leftFront , rightFront  , leftBack , rightBack;
    private double leftFrontPow , rightFrontPow , leftBackPow , rightBackPow;
    private double drive , strafe , turn;

    public Drive(DcMotorEx leftFront, DcMotorEx rightFront, DcMotorEx leftBack, DcMotorEx rightBack) {
        this.leftFront = leftFront;
        this.rightFront = rightFront;
        this.leftBack = leftBack;
        this.rightBack = rightBack;
    }

    public void stickInput(double drive , double strafe , double turn){
        final double MIN_POW = 0.1;
        this.drive = drive * (1 - MIN_POW) + Math.signum(drive) * drive;
    }

    @Override
    public void read(){

    }
    @Override
    public void loop(){
        leftFrontPow = drive + strafe + turn;
        leftBackPow = drive - strafe + turn;
        rightFrontPow = drive - strafe - turn;
        rightBackPow = drive + strafe - turn;



    }
    @Override
    public void write(){
        leftFront.setPower(leftFrontPow);
        leftBack.setPower(leftBackPow);
        rightFront.setPower(rightFrontPow);
        rightBack.setPower(rightBackPow);
    }
}
