package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;


@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class TeleOp extends LinearOpMode {
    RobotHardware robot = new RobotHardware(hardwareMap);
    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        robot.Init();
        waitForStart();

        while (opModeIsActive()) {
            robot.driveTrain.Drive();
            telemetry.update();
        }
    }
}

