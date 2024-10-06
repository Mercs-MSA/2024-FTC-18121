package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name="(SensorTest) Will It Crash?")

public class SensorDriveCode extends LinearOpMode {
    private DcMotor frontLeft = null;
    private DcMotor backLeft = null;
    private DcMotor frontRight = null;
    private DcMotor backRight = null;
    //private DcMotor imagine = null;
    private DistanceSensor distance;
    private ColorSensor color;
    private TouchSensor touch;
    public void intializeElectronics() {
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        //imagine = hardwareMap.get(DcMotor.class, "imagine");
        distance = hardwareMap.get(DistanceSensor.class, "Distance");
        color = hardwareMap.get(ColorSensor.class, "Color");
        touch = hardwareMap.get(TouchSensor.class, "Touch");
    }
    @Override
    public void runOpMode() {
        intializeElectronics();

        ElapsedTime runtime = new ElapsedTime();

        //imagine.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //imagine.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            double max;

            double axial = -gamepad1.left_stick_y;
            double lateral = gamepad1.left_stick_x;
            double yaw = gamepad1.right_stick_x;

            double frontLeftPower = axial + lateral + yaw;
            double frontRightPower = axial - lateral - yaw;
            double backLeftPower = axial - lateral + yaw;
            double backRightPower = axial + lateral - yaw;
            //double imaginePower = 0.5;

            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion.
            max = Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower));
            max = Math.max(max, Math.abs(backLeftPower));
            max = Math.max(max, Math.abs(backRightPower));
            //max = Math.max(max, Math.abs(imaginePower));

            if (max > 1.0) {
                frontLeftPower  /= max;
                frontLeftPower /= max;
                backLeftPower   /= max;
                backRightPower  /= max;
                //imaginePower  /= max;
            }

            // If the distance in centimeters is less than 10, set the power to 0.3
            if (distance.getDistance(DistanceUnit.INCH) < 10) {
                frontLeft.setPower(.8);
                telemetry.addData("DistanceIsMoreThanTen", "Negative");
            } else {
                telemetry.addData("DistanceIsMoreThanTen", "Positive");
            }

            // If the Magnetic Limit Switch is pressed, stop the motor
            if (touch.isPressed()) {
                telemetry.addData("MotorTouch", "Yeah");
            } else { // Otherwise, run the motor
                telemetry.addData("MotorTouch", "Nah");
            }

            if (gamepad1.a) {
                backLeft.setPower(.8);
                telemetry.addData("IsBackLeftMoving", "Yeah");
            }

            if (gamepad1.b) {
                backRight.setPower(.8);
                telemetry.addData("IsBackRightMoving", "Yeah");
            }

            if (gamepad1.x) {
                frontLeft.setPower(.8);
                telemetry.addData("IsFrontLeftMoving", "Yeah");
            }

            if (gamepad1.y) {
                frontRight.setPower(.8);
                telemetry.addData("IsFrontRightMoving", "Yeah");
            }

            frontLeft.setPower(frontLeftPower);
            frontRight.setPower(frontRightPower);
            backLeft.setPower(backLeftPower);
            backRight.setPower(backRightPower);
            //imagine.setPower(imaginePower);

            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", frontLeftPower, frontRightPower);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", backLeftPower, backRightPower);
            //telemetry.addData("Imaginary Motor", "%4.2f, %4.2f", imaginePower);
            telemetry.addData("Red", color.red());
            telemetry.addData("Green", color.green());
            telemetry.addData("Blue", color.blue());
            telemetry.update();
        }
    }
}