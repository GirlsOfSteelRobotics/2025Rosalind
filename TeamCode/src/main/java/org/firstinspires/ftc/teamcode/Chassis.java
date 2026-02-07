package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
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

public class Chassis {
    private final DcMotorEx frontLeft;
    private final DcMotorEx backLeft;
    private final DcMotorEx frontRight;
    private final DcMotorEx backRight;
    private Limelight3A limelight;

    IMU.Parameters myIMUparameters = new IMU.Parameters(
            new RevHubOrientationOnRobot(
                    RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                    RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD
            ));
    private final IMU imu;

    public Chassis(HardwareMap hm, Telemetry telemetry) {
        frontLeft = (DcMotorEx) hm.dcMotor.get("LF");
        backLeft = (DcMotorEx) hm.dcMotor.get("LB");
        frontRight = (DcMotorEx) hm.dcMotor.get("RF");
        backRight = (DcMotorEx) hm.dcMotor.get("RB");
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
//        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
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
                telemetry.addData("tx", result.getTx());
                telemetry.addData("ty", result.getTy());
                telemetry.addData("ta", result.getTa());
                telemetry.addData("Botpose", botpose.toString());
            }
        }
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

    public void moveForwardBad(int numTicks, Telemetry telem) {
        frontRight.setTargetPosition(frontRight.getCurrentPosition() + numTicks);
        frontLeft.setTargetPosition(frontLeft.getCurrentPosition() + numTicks);
        backLeft.setTargetPosition(backLeft.getCurrentPosition() + numTicks);
        backRight.setTargetPosition(backRight.getCurrentPosition() + numTicks);

        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontLeft.setPower(.2);
        frontRight.setPower(.2);
        backLeft.setPower(.2);
        backRight.setPower(.2);

        while (frontRight.isBusy() || frontLeft.isBusy() || backRight.isBusy() || backLeft.isBusy()) {
            telem.addData("isbusy", true);
            telem.addData("fl", frontLeft.getCurrentPosition());
            telem.addData("fr", frontRight.getCurrentPosition());
            telem.addData("bl", backLeft.getCurrentPosition());
            telem.addData("br", backRight.getCurrentPosition());
            telem.update();
        }

        // set motor power back to 0
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);

    }

    public void moveForward(int numTicks, Telemetry telem) {
        frontLeft.setTargetPositionTolerance(200);
        frontRight.setTargetPositionTolerance(200);
        backLeft.setTargetPositionTolerance(200);
        backRight.setTargetPositionTolerance(200);
        MotorGroup leftMotors = new MotorGroup(frontLeft, backLeft);
        MotorGroup rightMotors = new MotorGroup(frontRight, backRight);
        //x= input (inches)
        //distanceTicks = (1120x)/12.566
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
        frontLeft.setTargetPositionTolerance(-200);
        frontRight.setTargetPositionTolerance(-200);
        backLeft.setTargetPositionTolerance(-200);
        backRight.setTargetPositionTolerance(-200);
        MotorGroup leftMotors = new MotorGroup(frontLeft, backLeft);
        MotorGroup rightMotors = new MotorGroup(frontRight, backRight);
        //x= input (inches)
        //distanceTicks = (1120x)/12.566
        leftMotors.setIncreasedTargetPosition(numTicks);
        rightMotors.setIncreasedTargetPosition(numTicks);
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
    public void moveSideways(int numTicks, Telemetry telem) {
        frontLeft.setTargetPositionTolerance(-200);
        frontRight.setTargetPositionTolerance(200);
        backLeft.setTargetPositionTolerance(200);
        backRight.setTargetPositionTolerance(-200);
        MotorGroup leftMotors = new MotorGroup(frontLeft, backLeft);
        MotorGroup rightMotors = new MotorGroup(frontRight, backRight);
        //x= input (inches)
        //distanceTicks = (1120x)/12.566
        leftMotors.setIncreasedTargetPosition(numTicks);
        frontLeft.setTargetPosition(frontLeft.getCurrentPosition()-numTicks);
        backRight.setTargetPosition(backRight.getCurrentPosition()-numTicks);
        frontRight.setTargetPosition(frontRight.getCurrentPosition()+numTicks);
        backLeft.setTargetPosition(backLeft.getCurrentPosition()+numTicks);

        rightMotors.setIncreasedTargetPosition(numTicks);
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
    public void moveToOtherSide(int numTicks, Telemetry telem) {
        frontLeft.setTargetPositionTolerance(200);
        frontRight.setTargetPositionTolerance(-200);
        backLeft.setTargetPositionTolerance(-200);
        backRight.setTargetPositionTolerance(200);
        MotorGroup leftMotors = new MotorGroup(frontLeft, backLeft);
        MotorGroup rightMotors = new MotorGroup(frontRight, backRight);
        //x= input (inches)
        //distanceTicks = (1120x)/12.566
        leftMotors.setIncreasedTargetPosition(numTicks);
        rightMotors.setIncreasedTargetPosition(numTicks);
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
    public void turnOneWay(int numTicks, Telemetry telem) {
        frontLeft.setTargetPositionTolerance(-200);
        frontRight.setTargetPositionTolerance(200);
        backLeft.setTargetPositionTolerance(-200);
        backRight.setTargetPositionTolerance(200);
        MotorGroup leftMotors = new MotorGroup(frontLeft, backLeft);
        MotorGroup rightMotors = new MotorGroup(frontRight, backRight);
        //x= input (inches)
        //distanceTicks = (1120x)/12.566
        leftMotors.setDecreasedTargetPosition(numTicks);
        rightMotors.setIncreasedTargetPosition(numTicks);
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
    public void turnTheOtherWay(int numTicks, Telemetry telem) {
        frontLeft.setTargetPositionTolerance(200);
        frontRight.setTargetPositionTolerance(-200);
        backLeft.setTargetPositionTolerance(200);
        backRight.setTargetPositionTolerance(-200);
        MotorGroup leftMotors = new MotorGroup(frontLeft, backLeft);
        MotorGroup rightMotors = new MotorGroup(frontRight, backRight);
        //x= input (inches)
        //distanceTicks = (1120x)/12.566
        leftMotors.setIncreasedTargetPosition(numTicks);
        rightMotors.setDecreasedTargetPosition(numTicks);
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
}

