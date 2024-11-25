package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="cool")
public class ServoPositioner extends LinearOpMode {
    public Servo wrist = null;
    public void runOpMode() {
        waitForStart();
        while (opModeIsActive()) {
            wrist = hardwareMap.get(Servo.class, "wrist");
            wrist.setPosition(0.5);
        }
    }
}
