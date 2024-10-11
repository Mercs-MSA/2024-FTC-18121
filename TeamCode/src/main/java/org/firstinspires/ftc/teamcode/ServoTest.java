package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;

@TeleOp(name = "Subscribe to Jesus and get a free PS2")
public class ServoTest extends LinearOpMode {
    public Servo wrist = null;
    public CRServo hand = null;
    public void runOpMode() {
        wrist = hardwareMap.get(Servo.class,"wrist");
        hand = hardwareMap.get(CRServo.class, "hand");
        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.a) {
                telemetry.addData("Turned?", "Yes");
                wrist.setPosition(0.8);
            }
            else {
                telemetry.addData("Turned?", "No");
                wrist.setPosition(0.5);
            }
            if (gamepad1.b) {
                telemetry.addData("Intake?", "Out");
                hand.setDirection(DcMotorSimple.Direction.FORWARD);
                hand.setPower(1);
            }
            else if (gamepad1.y) {
                telemetry.addData("Intake?", "In");
                hand.setDirection(DcMotorSimple.Direction.REVERSE);
                hand.setPower(1);
            }
            else {
                telemetry.addData("Intake?", "No");
                hand.setPower(0);
            }
            telemetry.update();
        }
    }
}