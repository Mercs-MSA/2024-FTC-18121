package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;

@TeleOp(name="Furries are awesome UwU")

public class LimelightTest extends LinearOpMode{

    private DcMotor frontLeft = null;
    private DcMotor backLeft = null;
    private DcMotor frontRight = null;
    private DcMotor backRight = null;

    private Limelight3A limelight;
    IMU imu;

    @Override
    public void runOpMode() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        imu = hardwareMap.get(IMU.class, "IMU");

        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        ElapsedTime runtime = new ElapsedTime();

        telemetry.setMsTransmissionInterval(11);

        limelight.pipelineSwitch(0);

        limelight.start();

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        waitForStart();
        runtime.reset();
        boolean areThereTags = false;
        double degreesToRotate = 0;
        while (opModeIsActive()) {
            LLResult result = limelight.getLatestResult();
            YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();

            degreesToRotate = 0;
            
            telemetry.addData("Yaw (Z)", "%.2f Deg. (Heading)", orientation.getYaw(AngleUnit.DEGREES));
            if (result != null) {
                if (result.isValid()) {
                    Pose3D botpose = result.getBotpose();
                    telemetry.addData("tx", result.getTx());
                    telemetry.addData("ty", result.getTy());
                    telemetry.addData("Botpose", botpose.toString());

                    List<LLResultTypes.FiducialResult> aprilTags = result.getFiducialResults();
                    for (int i = 0; i < aprilTags.toArray().length; i++) {
                        areThereTags = true;
                        degreesToRotate = aprilTags.get(i).getTargetXDegrees();
                        telemetry.addData("Fiducial ID's", aprilTags.get(i).getFiducialId());
                        telemetry.addData("AprilTag Degrees", aprilTags.get(i).getTargetXDegrees());
                        telemetry.addData("Target Vector", aprilTags.get(i).getCameraPoseTargetSpace().toString());
                        break;
                    }
                    if (aprilTags.toArray().length == 0) {
                        areThereTags = false;
                    }
                }
            }

            double max;

            double axial = 0;
            double lateral = 0;
            double yaw = 0;

            if (gamepad1.a) {
                telemetry.addData("a", "a");
                if (areThereTags) {
                    telemetry.addData("wow there", "are tags");
                }

                if (degreesToRotate > 0 && areThereTags) {
                    yaw = .25;
                }
                else if (degreesToRotate < 0 && areThereTags) {
                    yaw = -.25;
                }
            }

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

            frontLeft.setPower(frontLeftPower);
            frontRight.setPower(frontRightPower);
            backLeft.setPower(backLeftPower);
            backRight.setPower(backRightPower);

            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", frontLeftPower, frontRightPower);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", backLeftPower, backRightPower);

            telemetry.update();
        }
    }
}
