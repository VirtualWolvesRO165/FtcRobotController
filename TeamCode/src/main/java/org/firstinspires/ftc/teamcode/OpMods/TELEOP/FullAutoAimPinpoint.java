//package org.firstinspires.ftc.teamcode.OpMods.TELEOP;
//
//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.canvas.Canvas;
//import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
//
//import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.CRServo;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.qualcomm.robotcore.hardware.IMU;
//
//import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
//
//import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
//import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
//
//@TeleOp(name = "FTC AutoAim + Dashboard", group = "FTC")
//public class FullAutoAimPinpoint extends OpMode {
//
//    // ================= DRIVE =================
//    DcMotorEx leftFront, rightFront, leftBack, rightBack;
//
//    // ================= TURETA =================
//    CRServo turret;
//
//    // ================= IMU =================
//    IMU turretIMU;
//
//    // ================= PINPOINT =================
//    GoBildaPinpointDriver pinpoint;
//
//    // ================= DASHBOARD =================
//    FtcDashboard dashboard;
//
//    // ================= COȘ =================
//    double blueBasketX =  136;
//    double blueBasketY = 136;
////135 128
//    double redBasketX = 8;
//    double redBasketY = 8;
////10 128
//    boolean isBlue = true;
//
//    // ================= PID =================
//    double kP = 0.7;
//    double kD = 0.08;
//
//    double lastError = 0;
//    double lastTime = 0;
//
//    double deadZone = Math.toRadians(0.4);
//    double minPower = 0.10;
//
//    @Override
//    public void init() {
//
//        // ===== DRIVE =====
//        leftFront = hardwareMap.get(DcMotorEx.class , "leftFront");
//        rightBack = hardwareMap.get(DcMotorEx.class , "rightFront");
//        leftBack = hardwareMap.get(DcMotorEx.class , "leftBack");
//        rightBack = hardwareMap.get(DcMotorEx.class , "rightBack");
//
//        leftBack.setDirection(DcMotor.Direction.REVERSE);
//        leftFront.setDirection(DcMotor.Direction.REVERSE);
//
//
//        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//        // ===== CR SERVO =====
//        turret = hardwareMap.get(CRServo.class, "shooterRotation");
//        turret.setPower(0);
//
//        // ===== TURET IMU =====
//        turretIMU = hardwareMap.get(IMU.class, "turretIMU");
//
//        IMU.Parameters imuParams = new IMU.Parameters(
//                new RevHubOrientationOnRobot(
//                        RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
//                        RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD
//                )
//        );
//        turretIMU.initialize(imuParams);
//
//        // ===== PINPOINT =====
//        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
//        //pinpoint.resetPosAndIMU();
////        pinpoint.setPosition(9,9,0); //X,Y,heading
//
//
//
//        // ===== DASHBOARD =====
//        dashboard = FtcDashboard.getInstance();
//        dashboard.setTelemetryTransmissionInterval(25);
//
//        lastTime = getRuntime();
//    }
//
//    @Override
//    public void loop() {
//
//        // ================= ALLIANCE =================
//        if (gamepad1.x) isBlue = true;
//        if (gamepad1.b) isBlue = false;
//
//        double basketX = isBlue ? blueBasketX : redBasketX;
//        double basketY = isBlue ? blueBasketY : redBasketY;
//
//        // ================= MECANUM =================
//           double y = -gamepad.left_stick_y; // Remember, Y stick is reversed!
//        double x = gamepad.left_stick_x;
//        double rx = gamepad.right_stick_x;
//
//        if (!gamepad.left_bumper) {
//            x /= 2;
//            y /= 2;
//        }
//        if (!gamepad.right_bumper) {
//            rx /= 2;
//        }
//
//        leftFront.setPower(y + x + rx);
//        leftBack.setPower(y - x + rx);
//        rightFront.setPower(y - x - rx);
//        rightBack.setPower(y + x - rx);
//    }
//
//        double max = Math.max(1.0,
//                Math.max(Math.abs(lf),
//                        Math.max(Math.abs(rf),
//                                Math.max(Math.abs(lb), Math.abs(rb)))));
//
//        leftFront.setPower(lf / max);
//        rightFront.setPower(rf / max);
//        leftBack.setPower(lb / max);
//        rightBack.setPower(rb / max);
//
//
////        leftFront.setPower((gamepad1.left_stick_y+gamepad1.left_stick_x+gamepad1.right_stick_x)*2);
////        leftBack.setPower((gamepad1.left_stick_y-gamepad1.left_stick_x+gamepad1.right_stick_x)*2);
////        rightFront.setPower((gamepad1.left_stick_y-gamepad1.left_stick_x-gamepad1.right_stick_x)*2);
////        rightBack.setPower((gamepad1.left_stick_y+gamepad1.left_stick_x-gamepad1.right_stick_x)*2);
//
//        // ================= PINPOINT =================
//        pinpoint.update();
//
//        double robotX = pinpoint.getPosX(DistanceUnit.INCH);
//        double robotY = pinpoint.getPosY(DistanceUnit.INCH);
//
//        // ================= TURET IMU (FIX) =================
//        double turretHeading = -turretIMU.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
//
//        // ================= AUTO AIM =================
//        double targetAngle = Math.atan2(basketY - robotY, basketX - robotX);
//        double error = normalize(targetAngle - turretHeading);
//
//        // ================= PID CRSERVO =================
//        double now = getRuntime();
//        double dt = now - lastTime;
//        lastTime = now;
//
//        double output = kP * error + kD * ((error - lastError) / dt);
//
//        if (Math.abs(error) < deadZone) {
//            turret.setPower(0);
//        } else {
//            output = clip(output, -0.5, 0.5);
//
//            if (Math.abs(output) < minPower)
//                output = Math.signum(output) * minPower;
//
//            turret.setPower(output);
//        }
//
//        lastError = error;
//
//        // ================= DASHBOARD DRAW =================
//        TelemetryPacket packet = new TelemetryPacket();
//        Canvas field = packet.fieldOverlay();
//
//        double ROBOT_RADIUS = 9; // inch
//
//        // Robot
//        field.setStroke("#00FF00");
//        field.strokeCircle(robotX, robotY, ROBOT_RADIUS);
//
//        double hx = robotX + ROBOT_RADIUS * Math.cos(turretHeading);
//        double hy = robotY + ROBOT_RADIUS * Math.sin(turretHeading);
//        field.strokeLine(robotX, robotY, hx, hy);
//
//        // Target
//        field.setStroke("#FF0000");
//        field.strokeCircle(basketX, basketY, 3);
//
//        // Robot -> Target
//        field.setStroke("#FFA500");
//        field.strokeLine(robotX, robotY, basketX, basketY);
//
//        // Values
//        packet.put("robotX", robotX);
//        packet.put("robotY", robotY);
//        packet.put("turretHeadingDeg", Math.toDegrees(turretHeading));
//        packet.put("targetAngleDeg", Math.toDegrees(targetAngle));
//        packet.put("errorDeg", Math.toDegrees(error));
//
//        dashboard.sendTelemetryPacket(packet);
//
//        // ================= DRIVER STATION TELEMETRY =================
//        telemetry.addData("Alliance", isBlue ? "BLUE" : "RED");
//        telemetry.addData("Error (deg)", Math.toDegrees(error));
//        telemetry.addData("Servo Power", turret.getPower());
//        telemetry.update();
//    }
//
//    // ================= UTILS =================
//    double clip(double v, double min, double max) {
//        return Math.max(min, Math.min(max, v));
//    }
//
//    double normalize(double a) {
//        while (a > Math.PI) a -= 2 * Math.PI;
//        while (a < -Math.PI) a += 2 * Math.PI;
//        return a;
//    }
//}
