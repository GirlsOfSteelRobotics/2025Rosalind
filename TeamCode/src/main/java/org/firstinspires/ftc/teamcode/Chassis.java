package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import dev.nextftc.control.KineticState;
import dev.nextftc.control.feedback.FeedbackType;
import dev.nextftc.control.feedback.PIDCoefficients;
import dev.nextftc.control.feedback.PIDElement;

import java.util.List;

public class Chassis {
    private final DcMotorEx frontLeft;
    private final DcMotorEx backLeft;
    private final DcMotorEx frontRight;
    private final DcMotorEx backRight;
    private final Limelight3A limelight;

    int lastLength = 0;
    double axial = 0.0;
    double lateral = 0.0;
    double yaw = 0.0;

    public static double axialOffset = 1.0;
    public static double lateralOffset = 0.2;
    public static double yawOffset = 0.0;

    private PIDElement yawPID;
    private PIDElement axialPID;
    private PIDElement lateralPID;

    public static PIDCoefficients yawPIDCoefficients = new PIDCoefficients(0.05,0.0,0.0);
    public static PIDCoefficients axialPIDCoefficients = new PIDCoefficients(0.9,0.0,0.0005);
    public static PIDCoefficients lateralPIDCoefficients = new PIDCoefficients(-0.3,0.0,0.00001);

    IMU.Parameters myIMUparameters = new IMU.Parameters(
            new RevHubOrientationOnRobot(
                    RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                    RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD
            ));
    private final IMU imu;
    private int tx;

    private int ty;

    public Chassis(HardwareMap hm, Telemetry telemetry) {
        yawPID = new PIDElement(FeedbackType.POSITION, yawPIDCoefficients);
        axialPID = new PIDElement(FeedbackType.POSITION, axialPIDCoefficients);
        lateralPID = new PIDElement(FeedbackType.POSITION, lateralPIDCoefficients);
        frontLeft = (DcMotorEx) hm.dcMotor.get("LF");
        backLeft = (DcMotorEx) hm.dcMotor.get("LB");
        frontRight = (DcMotorEx) hm.dcMotor.get("RF");
        backRight = (DcMotorEx) hm.dcMotor.get("RB");
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        imu = hm.get(IMU.class, "imu");
        imu.initialize(myIMUparameters);
        limelight = hm.get(Limelight3A.class, "limelight");
        telemetry.setMsTransmissionInterval(11);
        limelight.pipelineSwitch(0);
        limelight.start();
    }

    public void limeLight(Telemetry telemetry) {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        limelight.updateRobotOrientation(orientation.getYaw(AngleUnit.DEGREES));
        LLResult result = limelight.getLatestResult();
        if (result != null) {
            if (result.isValid()) {
                Pose3D botpose = result.getBotpose_MT2();
                telemetry.addData("botpose.pitch", botpose.getOrientation().getPitch());
                telemetry.addData("botpose.roll", botpose.getOrientation().getRoll());
                telemetry.addData("botpose.yaw", botpose.getOrientation().getYaw());
                telemetry.addData("tx", result.getTx());
                telemetry.addData("ty", result.getTy());
                telemetry.addData("ta", result.getTa());
                telemetry.addData("Botpose", botpose.toString());
            }
        }

        List<LLResultTypes.FiducialResult> fiducialResults = limelight.getLatestResult().getFiducialResults();

        if (!fiducialResults.isEmpty()) {
            LLResultTypes.FiducialResult snapshot = fiducialResults.get(0);

            //if (lastLength != fiducialResults.size()) {
            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            axial = -axialOffset - snapshot.getRobotPoseTargetSpace().getPosition().z; // Note: pushing stick forward gives negative value
            lateral = -lateralOffset + snapshot.getRobotPoseTargetSpace().getPosition().x;
            yaw = -yawOffset + snapshot.getRobotPoseTargetSpace().getOrientation().getYaw();
            telemetry.addData("Axial error:", -axialOffset - snapshot.getRobotPoseTargetSpace().getPosition().z);
            telemetry.addData("Lateral error:", -lateralOffset + snapshot.getRobotPoseTargetSpace().getPosition().x);
            telemetry.addData("Yaw error:", -yawOffset + snapshot.getRobotPoseTargetSpace().getOrientation().getYaw());
//            telemetry.addData("Axial output:", axial);
//            telemetry.addData("Lateral output:", lateral);
//            telemetry.addData("Yaw output:", yaw);
            telemetry.addData("Axial:", -snapshot.getRobotPoseTargetSpace().getPosition().z);
            telemetry.addData("Lateral:", snapshot.getRobotPoseTargetSpace().getPosition().x);
            telemetry.addData("Yaw:", snapshot.getRobotPoseTargetSpace().getOrientation().getYaw());
        }
//        if (tx>11){
//            turnRight(500, telemetry);
//        }else if(tx<11){
//            turnLeft(500, telemetry);
//        }ami
//        if(ty<0){
//            moveForward(500, telemetry);
//        }else if(ty>0){
//            moveBackwards(500, telemetry);
//        }
    }

    public void drive(double x, double y, double rx) {
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        // Rotate the movement direction counter to the bot's rotation
        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        rotX = rotX * 1.1;  // Counteract imperfect strafing

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        double frontLeftPower = (rotY + rotX + rx) / denominator;
        double backLeftPower = (rotY - rotX + rx) / denominator;
        double frontRightPower = (rotY - rotX - rx) / denominator;
        double backRightPower = (rotY + rotX - rx) / denominator;

        frontLeft.setPower(frontLeftPower);
        backLeft.setPower(backLeftPower);
        frontRight.setPower(frontRightPower);
        backRight.setPower(backRightPower);
    }

    public void resetIMU() {
        imu.resetYaw();
    }
    public void moveForward(int numTicks, Telemetry telem) {
        frontLeft.setTargetPositionTolerance(200);
        frontRight.setTargetPositionTolerance(200);
        backLeft.setTargetPositionTolerance(200);
        backRight.setTargetPositionTolerance(200);
        MotorGroup leftMotors = new MotorGroup(frontLeft, backLeft);
        MotorGroup rightMotors = new MotorGroup(frontRight, backRight);
        leftMotors.setIncreasedTargetPosition(numTicks);
        rightMotors.setIncreasedTargetPosition(numTicks);
        leftMotors.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightMotors.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightMotors.setPower(-1);
        leftMotors.setPower(-1);

        while ((leftMotors.isBusy() || rightMotors.isBusy())) {
            telem.addData("isbusy", true);
            telem.addData("fl", frontLeft.getCurrentPosition());
            telem.addData("fr", frontRight.getCurrentPosition());
            telem.addData("bl", backLeft.getCurrentPosition());
            telem.addData("br", backRight.getCurrentPosition());
            telem.update();
        }
        // set motor power back to 0
        leftMotors.setPower(0);
        rightMotors.setPower(0);
    }

    public void moveBackwards(int numTicks, Telemetry telem) {
        frontLeft.setTargetPositionTolerance(200);
        frontRight.setTargetPositionTolerance(200);
        backLeft.setTargetPositionTolerance(200);
        backRight.setTargetPositionTolerance(200);
        MotorGroup leftMotors = new MotorGroup(frontLeft, backLeft);
        MotorGroup rightMotors = new MotorGroup(frontRight, backRight);
        leftMotors.setDecreasedTargetPosition(numTicks);
        rightMotors.setDecreasedTargetPosition(numTicks);
        leftMotors.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightMotors.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightMotors.setPower(1);
        leftMotors.setPower(1);

        while ((leftMotors.isBusy() || rightMotors.isBusy())) {
            telem.addData("isbusy", true);
            telem.addData("fl", frontLeft.getCurrentPosition());
            telem.addData("fr", frontRight.getCurrentPosition());
            telem.addData("bl", backLeft.getCurrentPosition());
            telem.addData("br", backRight.getCurrentPosition());
            telem.update();
        }
        leftMotors.setPower(0);
        rightMotors.setPower(0);
    }
    public void sidewaysLeft(int numTicks, Telemetry telem) {
        frontLeft.setTargetPositionTolerance(200);
        frontRight.setTargetPositionTolerance(200);
        backLeft.setTargetPositionTolerance(200);
        backRight.setTargetPositionTolerance(200);
        MotorGroup leftMotors = new MotorGroup(frontLeft, backLeft);
        MotorGroup rightMotors = new MotorGroup(frontRight, backRight);
        frontLeft.setTargetPosition(frontLeft.getCurrentPosition() - numTicks);
        backRight.setTargetPosition(backRight.getCurrentPosition() - numTicks);
        frontRight.setTargetPosition(frontRight.getCurrentPosition() + numTicks);
        backLeft.setTargetPosition(backLeft.getCurrentPosition() + numTicks);

        leftMotors.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightMotors.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontLeft.setPower(1);
        backRight.setPower(1);
        frontRight.setPower(-1);
        backLeft.setPower(-1);

        while ((leftMotors.isBusy() || rightMotors.isBusy())) {
            telem.addData("isbusy", true);
            telem.addData("fl", frontLeft.getCurrentPosition());
            telem.addData("fr", frontRight.getCurrentPosition());
            telem.addData("bl", backLeft.getCurrentPosition());
            telem.addData("br", backRight.getCurrentPosition());
            telem.update();
        }
        leftMotors.setPower(0);
        rightMotors.setPower(0);
    }
    public void turnRight(int numTicks, Telemetry telem) {
        frontLeft.setTargetPositionTolerance(200);
        frontRight.setTargetPositionTolerance(200);
        backLeft.setTargetPositionTolerance(200);
        backRight.setTargetPositionTolerance(200);
        MotorGroup leftMotors = new MotorGroup(frontLeft, backLeft);
        MotorGroup rightMotors = new MotorGroup(frontRight, backRight);
        leftMotors.setIncreasedTargetPosition(numTicks);
        rightMotors.setDecreasedTargetPosition(numTicks);
        leftMotors.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightMotors.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontLeft.setPower(-1);
        backRight.setPower(-1);
        frontRight.setPower(1);
        backLeft.setPower(1);

        while ((leftMotors.isBusy() || rightMotors.isBusy())) {
            telem.addData("isbusy", true);
            telem.addData("fl", frontLeft.getCurrentPosition());
            telem.addData("fr", frontRight.getCurrentPosition());
            telem.addData("bl", backLeft.getCurrentPosition());
            telem.addData("br", backRight.getCurrentPosition());
            telem.update();
        }
        leftMotors.setPower(0);
        rightMotors.setPower(0);
    }
    public void sidewaysRight(int numTicks, Telemetry telem) {
        frontLeft.setTargetPositionTolerance(200);
        frontRight.setTargetPositionTolerance(200);
        backLeft.setTargetPositionTolerance(200);
        backRight.setTargetPositionTolerance(200);
        MotorGroup leftMotors = new MotorGroup(frontLeft, backLeft);
        MotorGroup rightMotors = new MotorGroup(frontRight, backRight);
        frontLeft.setTargetPosition(frontLeft.getCurrentPosition()+numTicks);
        backRight.setTargetPosition(backRight.getCurrentPosition()+numTicks);
        frontRight.setTargetPosition(frontRight.getCurrentPosition()-numTicks);
        backLeft.setTargetPosition(backLeft.getCurrentPosition()-numTicks);

        leftMotors.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightMotors.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightMotors.setPower(-1);
        leftMotors.setPower(1);

        while ((leftMotors.isBusy() || rightMotors.isBusy())) {
            telem.addData("isbusy", true);
            telem.addData("fl", frontLeft.getCurrentPosition());
            telem.addData("fr", frontRight.getCurrentPosition());
            telem.addData("bl", backLeft.getCurrentPosition());
            telem.addData("br", backRight.getCurrentPosition());
            telem.update();
        }
        // set motor power back to 0
        leftMotors.setPower(0);
        rightMotors.setPower(0);
    }
    public void turnLeft(int numTicks, Telemetry telem) {
        frontLeft.setTargetPositionTolerance(200);
        frontRight.setTargetPositionTolerance(200);
        backLeft.setTargetPositionTolerance(200);
        backRight.setTargetPositionTolerance(200);
        MotorGroup leftMotors = new MotorGroup(frontLeft, backLeft);
        MotorGroup rightMotors = new MotorGroup(frontRight, backRight);
        leftMotors.setDecreasedTargetPosition(numTicks);
        rightMotors.setIncreasedTargetPosition(numTicks);
        leftMotors.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightMotors.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightMotors.setPower(1);
        leftMotors.setPower(-1);

        while ((leftMotors.isBusy() || rightMotors.isBusy())) {
            telem.addData("isbusy", true);
            telem.addData("fl", frontLeft.getCurrentPosition());
            telem.addData("fr", frontRight.getCurrentPosition());
            telem.addData("bl", backLeft.getCurrentPosition());
            telem.addData("br", backRight.getCurrentPosition());
            telem.update();
        }

        // set motor power back to 0
        leftMotors.setPower(0);
        rightMotors.setPower(0);
    }
    public void aprilTagDrive(Telemetry telemetry) {
        List<LLResultTypes.FiducialResult> fiducialResults = limelight.getLatestResult().getFiducialResults();

        if (!fiducialResults.isEmpty()) {
            LLResultTypes.FiducialResult snapshot = fiducialResults.get(0);

            //if (lastLength != fiducialResults.size()) {
            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            axial = axialPID.calculate(new KineticState(-axialOffset - snapshot.getRobotPoseTargetSpace().getPosition().z)); // Note: pushing stick forward gives negative value
            lateral = lateralPID.calculate(new KineticState(-lateralOffset + snapshot.getRobotPoseTargetSpace().getPosition().x));
            yaw = yawPID.calculate(new KineticState(-yawOffset + snapshot.getRobotPoseTargetSpace().getOrientation().getYaw() -1.5));
            /*
            } else {
                axial = 0.0;
                lateral = 0.0;
                yaw = 0.0;
            }

             */
            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            double leftFrontPower = axial + lateral + yaw;
            double rightFrontPower = axial - lateral - yaw;
            double leftBackPower = axial - lateral + yaw;
            double rightBackPower = axial + lateral - yaw;

            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion.


            // Send calculated power to wheels
            frontLeft.setPower(leftFrontPower);
            frontRight.setPower(rightFrontPower);
            backLeft.setPower(leftBackPower);
            backRight.setPower(rightBackPower);

            telemetry.addData("Axial error:", -axialOffset - snapshot.getRobotPoseTargetSpace().getPosition().z);
            telemetry.addData("Lateral error:", -lateralOffset + snapshot.getRobotPoseTargetSpace().getPosition().x);
            telemetry.addData("Yaw error:", -yawOffset + snapshot.getRobotPoseTargetSpace().getOrientation().getYaw() - 1.5);
            telemetry.addData("Axial output:", axial);
            telemetry.addData("Lateral output:", lateral);
            telemetry.addData("Yaw output:", yaw);
            telemetry.addData("Axial:", -snapshot.getRobotPoseTargetSpace().getPosition().z);
            telemetry.addData("Lateral:", snapshot.getRobotPoseTargetSpace().getPosition().x);
            telemetry.addData("Yaw:", snapshot.getRobotPoseTargetSpace().getOrientation().getYaw());
        }
        lastLength = fiducialResults.size();
    }
}


// if tx too big, turn Right; if too small (neg), turn left
// if ty too small, forwards; if too big, backwards