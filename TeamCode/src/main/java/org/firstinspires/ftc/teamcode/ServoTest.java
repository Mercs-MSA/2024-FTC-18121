package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Servo;
@TeleOp(name = "Subscribe to Jesus and get a free PS2")
public class ServoTest extends LinearOpMode {
    public Servo wrist;
    public void initializeServo() {
        wrist = hardwareMap.get(Servo.class,"wrist");
    }
    public void runOpMode() {
        wrist.setPosition(0);
    }
}
