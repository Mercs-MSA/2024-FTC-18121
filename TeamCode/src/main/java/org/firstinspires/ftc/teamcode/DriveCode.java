package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp(name="Idk")

public class DriveCode extends LinearOpMode {
    private DcMotor frontLeft = null;
    private DcMotor backLeft = null;
    private DcMotor frontRight = null;
    private DcMotor backRight = null;
    private SparkFunOTOS myOtos = null;

    @Override
    public void runOpMode() {
        intializeElectronics();
        configureOtos();

        ElapsedTime runtime = new ElapsedTime();

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

            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion.
            max = Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower));
            max = Math.max(max, Math.abs(backLeftPower));
            max = Math.max(max, Math.abs(backRightPower));

            if (max > 1.0) {
                frontLeftPower  /= max;
                frontLeftPower /= max;
                backLeftPower   /= max;
                backRightPower  /= max;
            }

            // Get the latest position, which includes the x and y coordinates, plus the
            // heading angle
            SparkFunOTOS.Pose2D pos = myOtos.getPosition();

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

            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", frontLeftPower, frontRightPower);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", backLeftPower, backRightPower);
            telemetry.addLine("Press Y (triangle) on Gamepad to reset tracking");
            telemetry.addLine("Press X (square) on Gamepad to calibrate the IMU");
            telemetry.addLine();
            telemetry.addData("X coordinate", pos.x);
            telemetry.addData("Y coordinate", pos.y);
            telemetry.addData("Heading angle", pos.h);
            telemetry.update();
        }
    }

    public void intializeElectronics() {
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        myOtos = hardwareMap.get(SparkFunOTOS.class, "sensor_otos");
    }

    private void configureOtos() {
        telemetry.addLine("Configuring OTOS...");
        telemetry.update();

        // Set the desired units for linear and angular measurements. Can be either
        // meters or inches for linear, and radians or degrees for angular. If not
        // set, the default is inches and degrees. Note that this setting is not
        // persisted in the sensor, so you need to set at the start of all your
        // OpModes if using the non-default value.
        // myOtos.setLinearUnit(DistanceUnit.METER);
        myOtos.setLinearUnit(DistanceUnit.INCH);
        // myOtos.setAngularUnit(AnguleUnit.RADIANS);
        myOtos.setAngularUnit(AngleUnit.DEGREES);

        // Assuming you've mounted your sensor to a robot and it's not centered,
        // you can specify the offset for the sensor relative to the center of the
        // robot. The units default to inches and degrees, but if you want to use
        // different units, specify them before setting the offset! Note that as of
        // firmware version 1.0, these values will be lost after a power cycle, so
        // you will need to set them each time you power up the sensor. For example, if
        // the sensor is mounted 5 inches to the left (negative X) and 10 inches
        // forward (positive Y) of the center of the robot, and mounted 90 degrees
        // clockwise (negative rotation) from the robot's orientation, the offset
        // would be {-5, 10, -90}. These can be any value, even the angle can be
        // tweaked slightly to compensate for imperfect mounting (eg. 1.3 degrees).
        SparkFunOTOS.Pose2D offset = new SparkFunOTOS.Pose2D(-3.3125, 1.75, 0);
        myOtos.setOffset(offset);

        // Here we can set the linear and angular scalars, which can compensate for
        // scaling issues with the sensor measurements. Note that as of firmware
        // version 1.0, these values will be lost after a power cycle, so you will
        // need to set them each time you power up the sensor. They can be any value
        // from 0.872 to 1.127 in increments of 0.001 (0.1%). It is recommended to
        // first set both scalars to 1.0, then calibrate the angular scalar, then
        // the linear scalar. To calibrate the angular scalar, spin the robot by
        // multiple rotations (eg. 10) to get a precise error, then set the scalar
        // to the inverse of the error. Remember that the angle wraps from -180 to
        // 180 degrees, so for example, if after 10 rotations counterclockwise
        // (positive rotation), the sensor reports -15 degrees, the required scalar
        // would be 3600/3585 = 1.004. To calibrate the linear scalar, move the
        // robot a known distance and measure the error; do this multiple times at
        // multiple speeds to get an average, then set the linear scalar to the
        // inverse of the error. For example, if you move the robot 100 inches and
        // the sensor reports 103 inches, set the linear scalar to 100/103 = 0.971
        myOtos.setLinearScalar(0.9799016);
        myOtos.setAngularScalar(0.9961217);
        // 157.48 true measurement
        // 160.29
        // 161.07
        // 160.78

        //160.713333 avg
        //0.979901686267189

        // The IMU on the OTOS includes a gyroscope and accelerometer, which could
        // have an offset. Note that as of firmware version 1.0, the calibration
        // will be lost after a power cycle; the OTOS performs a quick calibration
        // when it powers up, but it is recommended to perform a more thorough
        // calibration at the start of all your OpModes. Note that the sensor must
        // be completely stationary and flat during calibration! When calling
        // calibrateImu(), you can specify the number of samples to take and whether
        // to wait until the calibration is complete. If no parameters are provided,
        // it will take 255 samples and wait until done; each sample takes about
        // 2.4ms, so about 612ms total
        myOtos.calibrateImu();

        // Reset the tracking algorithm - this resets the position to the origin,
        // but can also be used to recover from some rare tracking errors
        myOtos.resetTracking();

        // After resetting the tracking, the OTOS will report that the robot is at
        // the origin. If your robot does not start at the origin, or you have
        // another source of location information (eg. vision odometry), you can set
        // the OTOS location to match and it will continue to track from there.
        SparkFunOTOS.Pose2D currentPosition = new SparkFunOTOS.Pose2D(0, 0, 0);
        myOtos.setPosition(currentPosition);

        // Get the hardware and firmware version
        SparkFunOTOS.Version hwVersion = new SparkFunOTOS.Version();
        SparkFunOTOS.Version fwVersion = new SparkFunOTOS.Version();
        myOtos.getVersionInfo(hwVersion, fwVersion);

//        telemetry.addLine("OTOS configured! Press start to get position data!");
//        telemetry.addLine();
//        telemetry.addLine(String.format("OTOS Hardware Version: v%d.%d", hwVersion.major, hwVersion.minor));
//        telemetry.addLine(String.format("OTOS Firmware Version: v%d.%d", fwVersion.major, fwVersion.minor));
        telemetry.update();
    }
}