package org.firstinspires.ftc.teamcode.Mechanisms;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.controller.PIDController;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorGroup;

import org.firstinspires.ftc.teamcode.Misc.RobotConstants;
import org.firstinspires.ftc.teamcode.Misc.cachedMotor;

public class turretShooter extends SubsystemBase
{
    PIDController headingControl;
    PIDController speedControl;
    CRServo turret1;
    CRServo turret2;
    Servo hood;
    DcMotorEx shooter1;
    DcMotorEx shooter2;
    double rawCalcPower;
    double powerToSet;
    double turretPosition = 0;
    double hoodPosition = 0;
    double currentSpeed;
    double nominalVoltage = 12.00; //voltage at which the shooter was tuned
    public turretShooter(final HardwareMap hMap)
    {
        shooter1 = hMap.get(DcMotorEx.class, RobotConstants.first_shooter_motor_name);
        shooter2 = hMap.get(DcMotorEx.class, RobotConstants.second_shooter_motor_name);
        turret1 = hMap.get(CRServo.class, RobotConstants.left_turret_servo_name);
        turret2 = hMap.get(CRServo.class, RobotConstants.right_turret_servo_name);
        hood = hMap.get(Servo.class, RobotConstants.hood_servo_name);
        speedControl = new PIDController(RobotConstants.shooter_kP, 0, RobotConstants.shooter_kD);
    }
    public void periodic()
    {
        currentSpeed = shooter1.getVelocity();
        rawCalcPower = speedControl.calculate(currentSpeed);
        powerToSet = rawCalcPower + (Math.signum(rawCalcPower) * RobotConstants.shooter_kL) + RobotConstants.shooter_kFF;
        shooter1.setPower(powerToSet);
        shooter2.setPower(powerToSet);
        //turret1.setPosition(turretPosition * RobotConstants.turret_conversion_factor_DEGREES);
        //turret2.setPosition(turretPosition * RobotConstants.turret_conversion_factor_DEGREES + RobotConstants.offset_between_servos);
        hood.setPosition(hoodPosition);
    }
    public void setSpeed(double speed)
    {
        speedControl.setSetPoint(speed);
    }
    public boolean atSpeed()
    {
        return speedControl.atSetPoint();
    }
    public void setTurretPosition(double turretPosition)
    {
        this.turretPosition = turretPosition;
    }
    public void setHoodPosition(double hoodPosition)
    {
        this.hoodPosition = hoodPosition;
    }
    public void primeShooter(double speed, double turretPosition, double hoodPosition)
    {
        setSpeed(speed);
        setTurretPosition(turretPosition);
        setHoodPosition(hoodPosition);
    }

}