package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Shooter {
    private final DcMotorEx shooterWheel;
    private final DcMotor orangeWheels;
    private final DcMotor intakeMain;
    private final DcMotor secondIntake;

    double shooterGoalVelocity = 1310;
    double shooterAllowableError = 130;
    double kf = 0.00032;
    double kp = 0.0025 ;

    public Shooter(HardwareMap hm) {
        shooterWheel = (DcMotorEx)hm.get(DcMotor.class, "ShooterWheel");
        orangeWheels = hm.get(DcMotor.class, "feederWheel");
        intakeMain = hm.get(DcMotor.class, "IntakeFast");
        secondIntake = hm.get(DcMotor.class, "ballPusher");
        shooterWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        orangeWheels.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeMain.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        secondIntake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    public void Shoot(boolean rightTrigger, boolean leftBumper, boolean a, Telemetry telemetry) {

        shooterWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        orangeWheels.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeMain.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        secondIntake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        double shooterSpeed;
        double error;
        double power;
        shooterSpeed = ((DcMotorEx) shooterWheel).getVelocity();
        error = shooterGoalVelocity - shooterSpeed;
        power = shooterGoalVelocity * kf + error * kp;
        if (rightTrigger) {
            shooterWheel.setPower(power);
            if (leftBumper || Math.abs(error) < shooterAllowableError) {
                orangeWheels.setPower(1);
            } else {
                orangeWheels.setPower(0);
            }
            if (a || Math.abs(error)<shooterAllowableError) {
                secondIntake.setPower(1);
                intakeMain.setPower(1);
            } else {
                secondIntake.setPower(0);
                intakeMain.setPower(0);
            }
        }else{
            if(a){
                secondIntake.setPower(1);
                intakeMain.setPower(1);
            }else {
                secondIntake.setPower(0);
                intakeMain.setPower(0);
            }
            shooterWheel.setPower(0);
            orangeWheels.setPower(0);
        }
        telemetry.addData("velocity", shooterSpeed);
        telemetry.addData("error", error);
    }
    public void shootEncoderBased(int numTicks, Telemetry telem){
        MotorGroup intakes = new MotorGroup((DcMotorEx) intakeMain, (DcMotorEx) secondIntake);
        shooterWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intakes.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        orangeWheels.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        int targetPos = 5000;
        shooterWheel.setTargetPosition(targetPos);
            shooterWheel.setPower(0.7);
                intakes.setPower(1);
                    orangeWheels.setPower(1);

        while ((shooterWheel.isBusy() || intakes.isBusy() || orangeWheels.isBusy())) {
            telem.addData("isbusy", true);
            telem.addData("shooter", shooterWheel.getCurrentPosition());
            telem.addData("intake main", intakeMain.getCurrentPosition());
            telem.addData("orange wheels", orangeWheels.getCurrentPosition());
            telem.update();
        }

        // set motor power back to 0
        shooterWheel.setPower(0);
        intakes.setPower(0);
        orangeWheels.setPower(0);
    }
    public void intakeStart (){
        MotorGroup intakes = new MotorGroup((DcMotorEx) intakeMain, (DcMotorEx) secondIntake);
        intakes.setPower(1);
    }
    public void intakeStop(){
        MotorGroup intakes = new MotorGroup((DcMotorEx) intakeMain, (DcMotorEx) secondIntake);
        intakes.setPower(0);
    }
    }