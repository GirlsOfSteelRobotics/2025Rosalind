package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Shooter {
    private final DcMotor shooterWheel;
    private final DcMotor orangeWheels;
    private final DcMotor intakeMain;
    private final DcMotor secondIntake;

    double shooterGoalVelocity = 1310;
    double shooterAllowableError = 130;
    double kf = 0.00032;
    double kp = 0.0025 ;

    public Shooter(HardwareMap hm) {
        shooterWheel = hm.get(DcMotor.class, "ShooterWheel");
        orangeWheels = hm.get(DcMotor.class, "feederWheel");
        intakeMain = hm.get(DcMotor.class, "IntakeFast");
        secondIntake = hm.get(DcMotor.class, "ballPusher");
    }
    public void Shoot(boolean rightTrigger, boolean leftBumper, boolean a, Telemetry telemetry) {
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
}