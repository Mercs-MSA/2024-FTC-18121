package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;

@TeleOp(name = "Limb testing")
public class ServoTest extends LinearOpMode {
    public Servo wrist = null;
    public CRServo hand = null;
    public DcMotor shoulder = null;
    public void runOpMode() {
        wrist = hardwareMap.get(Servo.class,"wrist");
        hand = hardwareMap.get(CRServo.class, "hand");
        shoulder = hardwareMap.get(DcMotor.class, "shoulder");
        shoulder.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        shoulder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // Reset the motor encoder
        shoulder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // Turn the motor back on when we are done
        double position = shoulder.getCurrentPosition();
        // TODO ADD STUFF FROM THE WEBSITE TO THE CODE. THE CODE ABOVE IS WHERE I LEFT OFF.
        // TODO https://gm0.org/en/latest/docs/software/tutorials/encoders.html

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

            if (gamepad1.dpad_down) {
                shoulder.setTargetPosition(9);
            }
            if (gamepad1.dpad_up) {
                shoulder.setTargetPosition(0);
            }

            telemetry.update();
        }
    }
}