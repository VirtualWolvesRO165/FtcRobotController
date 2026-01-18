//package org.firstinspires.ftc.teamcode;
//
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.qualcomm.robotcore.hardware.Servo;
//
//@TeleOp(name="Test Servo")
//public class Test_servo extends OpMode {
//
//    private Servo testservo;
//    private DcMotorEx testmotor;
//
//    @Override
//    public void init() {
//        testservo = hardwareMap.get(Servo.class, "testservo");
//        testservo.setDirection(Servo.Direction.FORWARD);
//        testmotor  = hardwareMap.get(DcMotorEx.class, "testmotor");
//        testmotor.setDirection(DcMotorSimple.Direction.FORWARD);
//    }
//
//    @Override
//    public void loop() {
//
//        if (gamepad1.b)
//            testservo.setPosition(0);
//        if (gamepad1.x)
//            testservo.setPosition(0.5);
//        if (gamepad1.a)
//            testservo.setPosition(1);
//        if(gamepad1.left_trigger>0.1)
//            testmotor.setPower(1);
//        testmotor.setPower(0);
//        if(gamepad1.right_trigger>0.1)
//            testmotor.setPower(-1);
//                --+
//
//                testmotor.setPower(0);
//
//        telemetry.addData("Servo Position", testservo.getPosition());
//        telemetry.addData("Test Motor", testmotor.getPower());
//        telemetry.update();
//    }
//}
