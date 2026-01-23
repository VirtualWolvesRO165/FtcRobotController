package org.firstinspires.ftc.teamcode.OpMods.TELEOP;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "AutoAim Turret Motor NO IMU", group = "FTC")
public class FullAutoAimPinpoint extends OpMode {

    // ================= DRIVE =================
    DcMotorEx leftFront, rightFront, leftBack, rightBack;

    // ================= TURETA =================
    DcMotorEx turretMotor;


    // ================= PINPOINT =================
    GoBildaPinpointDriver pinpoint;

    // ================= DASHBOARD =================
    FtcDashboard dashboard;

    // ================= COȘ =================
    double blueBasketX = 136;
    double blueBasketY = 136;

    double redBasketX = 8;
    double redBasketY = 8;

    boolean isBlue = true;

    // ================= ENCODER (5203 YELLOW) =================
    static final double TICKS_PER_REV = 537.7;   // encoder intern
    static final double GEAR_RATIO = 19.2;       // raport cutie 5203 Yellow

    // ================= PID (5203 YELLOW) =================
    double kP = 0.35;
    double kD = 0.04;

    double lastError = 0;
    double lastTime = 0;

    double deadZone = Math.toRadians(0.3);
    double minPower = 0.07;

    // ================= ANTI CABLURI =================
    double lastTurretAngle = 0;
    double rotationCount = 0;
    static final double MAX_TOTAL_ROT = Math.toRadians(300);

    @Override
    public void init() {
        // ===== DRIVE =====
        leftFront  = hardwareMap.get(DcMotorEx.class, "leftFront");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        leftBack   = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightBack  = hardwareMap.get(DcMotorEx.class, "rightBack");

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);

        // ===== TURETĂ =====
        turretMotor = hardwareMap.get(DcMotorEx.class, "turretMotor");
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // ===== PINPOINT =====
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        // ===== DASHBOARD =====
        dashboard = FtcDashboard.getInstance();
        dashboard.setTelemetryTransmissionInterval(25);

        lastTime = getRuntime();
    }

    @Override
    public void loop() {
        // ================= ALLIANCE =================
        ///  if (gamepad1.x) isBlue = true;
        if (gamepad1.b) isBlue = false;

        double basketX = isBlue ? blueBasketX : redBasketX;
        double basketY = isBlue ? blueBasketY : redBasketY;

        // ================= DRIVE =================
        double y  = -gamepad1.left_stick_y;
        double x  =  gamepad1.left_stick_x;
        double rx =  gamepad1.right_stick_x;

        if (!gamepad1.left_bumper) {
            x *= 0.5;
            y *= 0.5;
        }
        if (!gamepad1.right_bumper) {
            rx *= 0.5;
        }

        double lf = y + x + rx;
        double rf = y - x - rx;
        double lb = y - x + rx;
        double rb = y + x - rx;

        double max = Math.max(1.0,
                Math.max(Math.abs(lf),
                        Math.max(Math.abs(rf),
                                Math.max(Math.abs(lb), Math.abs(rb)))));

        leftFront.setPower(lf / max);
        rightFront.setPower(rf / max);
        leftBack.setPower(lb / max);
        rightBack.setPower(rb / max);

        // ================= PINPOINT =================
        pinpoint.update();

        double robotX = pinpoint.getPosX(DistanceUnit.INCH);
        double robotY = pinpoint.getPosY(DistanceUnit.INCH);

        // ================= TURETĂ ANGLE =================
        double turretAngle = angleWrap(getTurretAngleRad());

        double delta = turretAngle - lastTurretAngle;
        if (delta > Math.PI)  rotationCount--;
        if (delta < -Math.PI) rotationCount++;
        lastTurretAngle = turretAngle;

        double totalAngle = rotationCount * 2 * Math.PI + turretAngle;

        // ================= AUTO AIM =================
        double targetAngle = Math.atan2(basketY - robotY, basketX - robotX);
        double error = angleWrap(targetAngle - turretAngle);

        if (Math.abs(totalAngle) > MAX_TOTAL_ROT) {
            error = angleWrap(error + Math.PI);
        }

        // ================= PID =================
        double now = getRuntime();
        double dt = now - lastTime;
        lastTime = now;

        double output = kP * error + kD * ((error - lastError) / dt);
        output = clip(output, -0.6, 0.6);

        if (Math.abs(error) < deadZone) {
            turretMotor.setPower(0);
        } else {
            if (Math.abs(output) < minPower)
                output = Math.signum(output) * minPower;

            turretMotor.setPower(output);
        }

        lastError = error;

        // ================= DASHBOARD =================
        TelemetryPacket packet = new TelemetryPacket();
        Canvas field = packet.fieldOverlay();

        field.setStroke("#00FF00");
        field.strokeCircle(robotX, robotY, 9);

        field.setStroke("#FF0000");
        field.strokeCircle(basketX, basketY, 3);

        dashboard.sendTelemetryPacket(packet);

        // ================= TELEMETRY =================
        telemetry.addData("Alliance", isBlue ? "BLUE" : "RED");
        telemetry.addData("Turret Angle (deg)", Math.toDegrees(turretAngle));
        telemetry.addData("Total Rot (deg)", Math.toDegrees(totalAngle));
        telemetry.update();
    }

    // ================= UTILS =================
    double getTurretAngleRad() {
        double ticks = turretMotor.getCurrentPosition();
        return (ticks / (TICKS_PER_REV * GEAR_RATIO)) * 2.0 * Math.PI;
    }

    double clip(double v, double min, double max) {
        return Math.max(min, Math.min(max, v));
    }

    double angleWrap(double a) {
        while (a > Math.PI) a -= 2 * Math.PI;
        while (a < -Math.PI) a += 2 * Math.PI;
        return a;
    }
}
