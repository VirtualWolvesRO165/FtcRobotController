package org.firstinspires.ftc.teamcode;

import com.google.ar.core.Config;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

public class RobotHardware {

    public DcMotorEx leftFront , rightFront , leftBack , rightBack;

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

        // This uses RUN_USING_ENCODER to be more accurate.   If you don't have the encoder
        // wires, you should remove these
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        imu = hardwareMap.get(IMU.class, "imu");
        // This needs to be changed to match the orientation on your robot
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection =
                RevHubOrientationOnRobot.LogoFacingDirection.DOWN;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection =
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;

        RevHubOrientationOnRobot orientationOnRobot = new
                RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        driveTrain.Init(leftFront , leftBack , rightFront , rightBack);
        /// DRIVETRAIN
    }

    public void Drive(){
        driveTrain.Drive();
    }

}
