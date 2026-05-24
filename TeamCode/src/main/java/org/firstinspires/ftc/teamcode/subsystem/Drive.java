package org.firstinspires.ftc.teamcode.subsystem;

import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.pedroCommand.TurnCommand;

import static org.firstinspires.ftc.teamcode.robot.Constants.DRIVE_MULTIPLIER;
import static org.firstinspires.ftc.teamcode.robot.Constants.IS_BEFORE_IN_CLOSE;
import static org.firstinspires.ftc.teamcode.robot.Constants.IS_BEFORE_IN_FAR;
import static org.firstinspires.ftc.teamcode.robot.Constants.IS_FULL;
import static org.firstinspires.ftc.teamcode.robot.Constants.ROBOT_RADIUS;
import static org.firstinspires.ftc.teamcode.robot.Constants.ROBOT_X;
import static org.firstinspires.ftc.teamcode.robot.Constants.ROBOT_Y;
import static org.firstinspires.ftc.teamcode.robot.Constants.IS_IN_FAR;
import static org.firstinspires.ftc.teamcode.robot.Constants.IS_IN_CLOSE;
import static org.firstinspires.ftc.teamcode.robot.Constants.IS_IN_PARKING;
import static org.firstinspires.ftc.teamcode.robot.Constants.alliance;
import static org.firstinspires.ftc.teamcode.robot.Constants.endGame;
import static org.firstinspires.ftc.teamcode.robot.Constants.enteredLaunchZone;
import static org.firstinspires.ftc.teamcode.robot.Constants.findParking;
import static org.firstinspires.ftc.teamcode.robot.Constants.leavingLaunchZone;

import android.net.wifi.aware.ParcelablePeerHandle;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.robot.Constants;
import org.firstinspires.ftc.teamcode.robot.Robot;

import java.util.List;

public class Drive extends SubsystemBase {
    private final Robot robot = Robot.getInstance();

    public enum DriveMode{
        PARKING , MAXIMUM
    }

    private Polygon farZone = new Polygon(List.of(
            new Point(49, 0),
            new Point(72, 23),
            new Point(95, 0)
    ));

    private Polygon beforeFarZone = new Polygon(List.of(
            new Point(34 , 0),
            new Point(72 , 39),
            new Point(110 , 0)
    ));

    private Polygon closeZone = new Polygon(List.of(
            new Point(72, 72),
            new Point(15, 128),
            new Point(26, 144),
            new Point(119, 144),
            new Point(130, 128)
    ));

    private Polygon beforeCloseZone = new Polygon(List.of(
            new Point(72 , 62),
            new Point(10 , 122),
            new Point(26 , 144),
            new Point(119 , 144),
            new Point(135 , 122)
    ));

    private Polygon redParkingZone = new Polygon(List.of(
            new Point(30 , 42),
            new Point(48 , 42),
            new Point(48 , 24),
            new Point(30 , 24)
    ));
    private Polygon blueParkingZone = new Polygon(List.of(
            new Point(96 , 42),
            new Point(114 , 42),
            new Point(114 , 24),
            new Point(96, 24)
    ));

    public static DriveMode driveMode = DriveMode.MAXIMUM;

    public void init() {
    }

    public void Update(double p_drive, double strafe, double turn) {
        strafe = strafe * 1.1;  // Counteract imperfect strafing
        double denominator = Math.max(Math.abs(p_drive) + Math.abs(strafe) + Math.abs(turn), 1);
        robot.leftFront.setPower((p_drive + strafe + turn)/denominator * DRIVE_MULTIPLIER);
        robot.leftBack.setPower((p_drive - strafe + turn)/denominator * DRIVE_MULTIPLIER);
        robot.rightFront.setPower((p_drive - strafe - turn)/denominator * DRIVE_MULTIPLIER);
        robot.rightBack.setPower((p_drive + strafe - turn)/denominator * DRIVE_MULTIPLIER);

        robot.follower.update();
        ROBOT_X = robot.follower.getPose().getX();
        ROBOT_Y = robot.follower.getPose().getY();
        IS_IN_FAR = isInFar(ROBOT_X, ROBOT_Y, ROBOT_RADIUS, farZone.vertices.get(0), farZone.vertices.get(1), farZone.vertices.get(2));
        IS_IN_CLOSE = isInClose(ROBOT_X, ROBOT_Y, ROBOT_RADIUS, closeZone.vertices.get(0), closeZone.vertices.get(1), closeZone.vertices.get(2), closeZone.vertices.get(3), closeZone.vertices.get(4));
        IS_BEFORE_IN_FAR = isBeforeInFar(ROBOT_X, ROBOT_Y, ROBOT_RADIUS, beforeFarZone.vertices.get(0), beforeFarZone.vertices.get(1), beforeFarZone.vertices.get(2));
        IS_BEFORE_IN_CLOSE = isBeforeInClose(ROBOT_X, ROBOT_Y, ROBOT_RADIUS, beforeCloseZone.vertices.get(0), beforeCloseZone.vertices.get(1), beforeCloseZone.vertices.get(2), beforeCloseZone.vertices.get(3), beforeCloseZone.vertices.get(4));
        if (alliance == Constants.Alliance.BLUE)
            IS_IN_PARKING = isInBlueParking(ROBOT_X, ROBOT_Y, ROBOT_RADIUS, blueParkingZone.vertices.get(0), blueParkingZone.vertices.get(1), blueParkingZone.vertices.get(2), blueParkingZone.vertices.get(3));
        else
            IS_IN_PARKING = isInRedParking(ROBOT_X, ROBOT_Y, ROBOT_RADIUS, redParkingZone.vertices.get(0), redParkingZone.vertices.get(1), redParkingZone.vertices.get(2), redParkingZone.vertices.get(3));

        switch (driveMode){
            case PARKING:
                DRIVE_MULTIPLIER=0.15;
                break;
            case MAXIMUM:
                DRIVE_MULTIPLIER=1;
                break;
        }
//        if (runTime.seconds() >= 140 && !endGameRumble) {
//            driver.runRumbleEffect(endGame);
//            operator.runRumbleEffect(endGame);
//            endGameRumble = true;
//        }
//        if (runTime.seconds() >= 150 && !parkRumble) {
//            driver.runRumbleEffect(findParking);
//            parkRumble = true;
//        }
//        if (IS_IN_CLOSE && !launchZoneRumble) {
//            driver.runRumbleEffect(enteredLaunchZone);
//            operator.runRumbleEffect(enteredLaunchZone);
//            launchZoneRumble=true;
//        }
//        if(!IS_IN_CLOSE && launchZoneRumble) {
//            driver.runRumbleEffect(leavingLaunchZone);
//            operator.runRumbleEffect(leavingLaunchZone);
//            launchZoneRumble=false;
//
//        }
//        if (IS_IN_FAR && !launchZoneRumble) {
//            driver.runRumbleEffect(enteredLaunchZone);
//            operator.runRumbleEffect(enteredLaunchZone);
//            launchZoneRumble=true;
//        }
//        if(!IS_IN_FAR && launchZoneRumble) {
//            driver.runRumbleEffect(leavingLaunchZone);
//            operator.runRumbleEffect(leavingLaunchZone);
//            launchZoneRumble=false;
//        }
    }

    public void ToggleDrive(){
        if(driveMode == DriveMode.MAXIMUM){
            driveMode = DriveMode.PARKING;
            return;
        }
        if(driveMode == DriveMode.PARKING){
            driveMode = DriveMode.MAXIMUM;
        }
    }
    public double cross(double ax, double ay, double bx, double by, double px, double py) {
        return (bx - ax) * (py - ay) - (by - ay) * (px - ax);
    }

    public boolean pointIn3Polygon(double px, double py, Point a, Point b, Point c) {
        double c1 = cross(a.x, a.y, b.x, b.y, px, py);
        double c2 = cross(b.x, b.y, c.x, c.y, px, py);
        double c3 = cross(c.x, c.y, a.x, a.y, px, py);

        return (c1 >= 0 && c2 >= 0 && c3 >= 0) ||
                (c1 <= 0 && c2 <= 0 && c3 <= 0);
    }
    public boolean pointIn4Polygon(double px, double py, Point a, Point b, Point c , Point d) {
        double c1 = cross(a.x, a.y, b.x, b.y, px, py);
        double c2 = cross(b.x, b.y, c.x, c.y, px, py);
        double c3 = cross(c.x, c.y, d.x, d.y, px, py);
        double c4 = cross(d.x, d.y, a.x, a.y, px, py);

        return (c1 >= 0 && c2 >= 0 && c3 >= 0 && c4>=0) ||
                (c1 <= 0 && c2 <= 0 && c3 <= 0 && c4<=0);
    }

    public boolean pointIn5Polygon(double px, double py, Point a, Point b, Point c, Point d, Point e) {

        double c1 = cross(a.x, a.y, b.x, b.y, px, py);
        double c2 = cross(b.x, b.y, c.x, c.y, px, py);
        double c3 = cross(c.x, c.y, d.x, d.y, px, py);
        double c4 = cross(d.x, d.y, e.x, e.y, px, py);
        double c5 = cross(e.x, e.y, a.x, a.y, px, py);

        return (c1 >= 0 && c2 >= 0 && c3 >= 0 && c4 >= 0 && c5 >= 0) ||
                (c1 <= 0 && c2 <= 0 && c3 <= 0 && c4 <= 0 && c5 <= 0);
    }

    public double distanceToSegment(double px, double py, double x1, double y1, double x2, double y2) {

        double dx = x2 - x1;
        double dy = y2 - y1;

        double t = ((px - x1) * dx + (py - y1) * dy) / (dx * dx + dy * dy);

        t = Math.max(0, Math.min(1, t));
        double cx = x1 + t * dx;
        double cy = y1 + t * dy;

        return Math.hypot(px - cx, py - cy);
    }

    public boolean isInFar(double px, double py, double r, Point a, Point b, Point c) {

        if (pointIn3Polygon(px, py, a, b, c))
            return true;

        return distanceToSegment(px, py, a.x, a.y, b.x, b.y) <= r ||
                distanceToSegment(px, py, b.x, b.y, c.x, c.y) <= r ||
                distanceToSegment(px, py, c.x, c.y, a.x, a.y) <= r;
    }

    public boolean isBeforeInFar(double px, double py, double r, Point a, Point b, Point c){
        if (pointIn3Polygon(px, py, a, b, c))
            return true;

        return distanceToSegment(px, py, a.x, a.y, b.x, b.y) <= r ||
                distanceToSegment(px, py, b.x, b.y, c.x, c.y) <= r ||
                distanceToSegment(px, py, c.x, c.y, a.x, a.y) <= r;
    }

    public boolean isInClose(double px, double py, double r, Point a, Point b, Point c, Point d, Point e) {

        if (pointIn5Polygon(px, py, a, b, c, d, e))
            return true;

        return distanceToSegment(px, py, a.x, a.y, b.x, b.y) <= r ||
                distanceToSegment(px, py, b.x, b.y, c.x, c.y) <= r ||
                distanceToSegment(px, py, c.x, c.y, d.x, d.y) <= r ||
                distanceToSegment(px, py, d.x, d.y, e.x, e.y) <= r ||
                distanceToSegment(px, py, e.x, e.y, a.x, a.y) <= r;
    }

    public boolean isBeforeInClose(double px, double py, double r, Point a, Point b, Point c, Point d, Point e){
        if (pointIn5Polygon(px, py, a, b, c, d, e))
            return true;

        return distanceToSegment(px, py, a.x, a.y, b.x, b.y) <= r ||
                distanceToSegment(px, py, b.x, b.y, c.x, c.y) <= r ||
                distanceToSegment(px, py, c.x, c.y, d.x, d.y) <= r ||
                distanceToSegment(px, py, d.x, d.y, e.x, e.y) <= r ||
                distanceToSegment(px, py, e.x, e.y, a.x, a.y) <= r;
    }


    public boolean isInRedParking(double px, double py, double r, Point a, Point b, Point c, Point d) {

        if (pointIn4Polygon(px, py, a, b, c, d))
            return true;

        return distanceToSegment(px, py, a.x, a.y, b.x, b.y) <= 0 ||
                distanceToSegment(px, py, b.x, b.y, c.x, c.y) <= 0 ||
                distanceToSegment(px, py, c.x, c.y, d.x, d.y) <= 0 ||
                distanceToSegment(px, py, d.x, d.y, a.x, a.y) <= 0;
    }
    public boolean isInBlueParking(double px, double py, double r, Point a, Point b, Point c, Point d) {

        if (pointIn4Polygon(px, py, a, b, c, d))
            return true;

        return distanceToSegment(px, py, a.x, a.y, b.x, b.y) <= 0 ||
                distanceToSegment(px, py, b.x, b.y, c.x, c.y) <= 0 ||
                distanceToSegment(px, py, c.x, c.y, d.x, d.y) <= 0 ||
                distanceToSegment(px, py, d.x, d.y, a.x, a.y) <= 0;
    }

    public void ResetPositionBlue(){
        robot.follower.setPose(new Pose(23.9 , 125));
        robot.follower.setHeading(2.44);
    }
    public void ResetPositionRed(){
        robot.follower.setPose(new Pose(120.6 , 125));
        robot.follower.setHeading(0.66);
    }


}

class Polygon{
    public List<Point> vertices;

    public Polygon(List<Point> vertices){
        this.vertices = vertices;
    }
}

class Point {
    public double x, y;

    public Point(double x, double y) {
        this.x = x;
        this.y = y;
    }
}