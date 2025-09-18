package org.firstinspires.ftc.teamcode.old;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


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

