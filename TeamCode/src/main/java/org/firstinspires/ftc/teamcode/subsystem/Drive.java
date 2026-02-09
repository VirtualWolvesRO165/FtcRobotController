package org.firstinspires.ftc.teamcode.subsystem;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import static org.firstinspires.ftc.teamcode.robot.Constants.ENTRY_MARGIN;
import static org.firstinspires.ftc.teamcode.robot.Constants.ROBOT_RADIUS;
import static org.firstinspires.ftc.teamcode.robot.Constants.ROBOT_X;
import static org.firstinspires.ftc.teamcode.robot.Constants.ROBOT_Y;
import static org.firstinspires.ftc.teamcode.robot.Constants.IS_IN_FAR;
import static org.firstinspires.ftc.teamcode.robot.Constants.IS_IN_CLOSE;
import static org.firstinspires.ftc.teamcode.robot.Constants.SHOOTER_RPM_OFFSET;

import org.firstinspires.ftc.teamcode.robot.Robot;

import java.util.List;

public class Drive extends SubsystemBase {
    private final Robot robot = Robot.getInstance();

    private Polygon farZone = new Polygon(List.of(
            new Point(49, 0),
            new Point(72, 23),
            new Point(95, 0)
    ));

    private Polygon closeZone = new Polygon(List.of(
            new Point(72, 72),
            new Point(15, 128),
            new Point(26, 144),
            new Point(119, 144),
            new Point(130, 128)
    ));

    public void init() {
    }

    public void Update(double p_drive, double strafe, double turn) {
        robot.leftFront.setPower((p_drive + strafe + turn));
        robot.leftBack.setPower((p_drive - strafe + turn));
        robot.rightFront.setPower((p_drive - strafe - turn));
        robot.rightBack.setPower((p_drive + strafe - turn));

        robot.follower.update();
        ROBOT_X = robot.follower.getPose().getX();
        ROBOT_Y = robot.follower.getPose().getY();
        IS_IN_FAR = isInFar(ROBOT_X, ROBOT_Y, ROBOT_RADIUS, farZone.vertices.get(0), farZone.vertices.get(1), farZone.vertices.get(2));
        IS_IN_CLOSE = isInClose(ROBOT_X, ROBOT_Y, ROBOT_RADIUS, closeZone.vertices.get(0), closeZone.vertices.get(1), closeZone.vertices.get(2), closeZone.vertices.get(3), closeZone.vertices.get(4));
//        if((IS_IN_FAR || IS_IN_CLOSE) && robot.robotState == Robot.RobotState.POSITIONING)
//            robot.robotState = Robot.RobotState.SHOOTING;
        if(IS_IN_FAR)
            SHOOTER_RPM_OFFSET=400;
        if(IS_IN_CLOSE)
            SHOOTER_RPM_OFFSET=-100;
    }

    public double cross(double ax, double ay, double bx, double by, double px, double py) {
        return (bx - ax) * (py - ay) - (by - ay) * (px - ax);
    }

    public boolean pointInFar(double px, double py, Point a, Point b, Point c) {
        double c1 = cross(a.x, a.y, b.x, b.y, px, py);
        double c2 = cross(b.x, b.y, c.x, c.y, px, py);
        double c3 = cross(c.x, c.y, a.x, a.y, px, py);

        return (c1 >= 0 && c2 >= 0 && c3 >= 0) ||
                (c1 <= 0 && c2 <= 0 && c3 <= 0);
    }


    public boolean pointInClose(double px, double py,
                                Point a, Point b, Point c, Point d, Point e) {

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

    public boolean isInFar(double px, double py, double r,
                           Point a, Point b, Point c) {

        // Center inside
        if (pointInFar(px, py, a, b, c))
            return true;

        // Barely touching edges
        return distanceToSegment(px, py, a.x, a.y, b.x, b.y) <= r ||
                distanceToSegment(px, py, b.x, b.y, c.x, c.y) <= r ||
                distanceToSegment(px, py, c.x, c.y, a.x, a.y) <= r;
    }



    public boolean isInClose(double px, double py, double r,
                             Point a, Point b, Point c, Point d, Point e) {

        // Center inside
        if (pointInClose(px, py, a, b, c, d, e))
            return true;

        // Barely touching edges
        return distanceToSegment(px, py, a.x, a.y, b.x, b.y) <= r ||
                distanceToSegment(px, py, b.x, b.y, c.x, c.y) <= r ||
                distanceToSegment(px, py, c.x, c.y, d.x, d.y) <= r ||
                distanceToSegment(px, py, d.x, d.y, e.x, e.y) <= r ||
                distanceToSegment(px, py, e.x, e.y, a.x, a.y) <= r;
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