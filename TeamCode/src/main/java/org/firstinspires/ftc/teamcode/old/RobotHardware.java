package org.firstinspires.ftc.teamcode.old;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

public class RobotHardware {

    public DcMotorEx leftFront , rightFront , leftBack , rightBack;

    public Limelight3A limelight3A;
    public MyLimelight myLimelight;

    DriveTrain driveTrain = new DriveTrain();
    IMU imu;
    public RobotHardware(HardwareMap hardwareMap){
        /// DRIVETRAIN
        leftFront = hardwareMap.get(DcMotorEx.class , "leftFront");
        rightBack = hardwareMap.get(DcMotorEx.class , "rightFront");
        leftBack = hardwareMap.get(DcMotorEx.class , "leftBack");
        rightBack = hardwareMap.get(DcMotorEx.class , "rightBack");

        leftBack.setDirection(DcMotor.Direction.REVERSE);
        leftFront.setDirection(DcMotor.Direction.REVERSE);

        
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection =
                RevHubOrientationOnRobot.LogoFacingDirection.DOWN;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection =
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;

        RevHubOrientationOnRobot orientationOnRobot = new
                RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        driveTrain.Init(leftFront , leftBack , rightFront , rightBack);
        /// DRIVETRAIN

        /// Limelight
        limelight3A  = hardwareMap.get(Limelight3A.class , "Limelight");
        limelight3A.pipelineSwitch(0);
    }

    public void Init(){
        driveTrain.Init(leftFront , rightFront , leftBack , rightBack);
    }

    public void AutoInit(){
        driveTrain.Init(leftFront , rightFront , leftBack , rightBack);
        myLimelight.Init(limelight3A);

    }

}
