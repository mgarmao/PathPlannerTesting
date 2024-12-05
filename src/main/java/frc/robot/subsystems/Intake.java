// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Constants;


public class Intake extends SubsystemBase {
    // private final CANcoder motor_cc = new CANcoder(0);
    private final TalonFX motor = new TalonFX(1);
    // private final DutyCycleOut m_dutycycle = new DutyCycleOut(0);
    
    private CANSparkMax rotator1;
    private CANSparkMax rotator2;

    public Intake() {
        /** Create a new object to control the SPARK MAX motor controllers. */
        CurrentLimitsConfigs configs = new CurrentLimitsConfigs().withSupplyCurrentLimit(Constants.IndexerAmpLimit).withSupplyCurrentLimitEnable(true);

        motor.getConfigurator().apply(new TalonFXConfiguration().withCurrentLimits(configs));
        motor.setNeutralMode(NeutralModeValue.Brake);
        motor.setInverted(false);

        rotator1 = new CANSparkMax(Constants.rotator1, MotorType.kBrushless);
        rotator2 = new CANSparkMax(Constants.rotator2, MotorType.kBrushless);

        rotator1.restoreFactoryDefaults();
        rotator2.restoreFactoryDefaults();


        
        rotator1.setSmartCurrentLimit(Constants.RotatorAmpLimit);
        rotator2.setSmartCurrentLimit(Constants.RotatorAmpLimit);

        /**
         * When the SPARK MAX is receiving a neutral command, the idle behavior of the motor 
         * will effectively disconnect all motor wires. This allows the motor to spin down at 
         * its own rate. 
         */
        rotator1.setIdleMode(IdleMode.kBrake);
        rotator2.setIdleMode(IdleMode.kBrake);

        rotator1.setSoftLimit(SoftLimitDirection.kForward, Constants.rotatorUpperLimit);
        rotator2.setSoftLimit(SoftLimitDirection.kForward, Constants.rotatorUpperLimit);
        rotator1.setSoftLimit(SoftLimitDirection.kReverse,Constants.rotatorLowerLimit);
        rotator2.setSoftLimit(SoftLimitDirection.kReverse,Constants.rotatorLowerLimit);
        
        rotator1.enableSoftLimit(SoftLimitDirection.kReverse, false);
        rotator1.enableSoftLimit(SoftLimitDirection.kForward, false);

        rotator2.enableSoftLimit(SoftLimitDirection.kForward, false);
        rotator2.enableSoftLimit(SoftLimitDirection.kReverse, false);

        rotator2.follow(rotator1);
        
        SignalLogger.enableAutoLogging(false);
    }

    /** Retrieve cargo for transportation. 
     * @return */
    public Command run() {
        return runOnce(
        ()->{
            motor.set(1);
        });
    }

    public void set(double speed) {
        motor.set(speed);
    }

    public Command reverse() {
        return runOnce(
        ()->{
            motor.set(-1);
        });
    }

    /** Eject cargo from the robot. */
    public Command stop() {
        return runOnce(
        ()->{
            motor.stopMotor();
        });
    }

    public Command extend(){
        return runOnce(
        ()->{
            rotator1.set(0.4);
        });
    }
    

    public Command retract(){
        return runOnce(
        ()->{
            rotator1.set(-0.4);
        });
    }

    public void stopExtendRetract(){

        rotator1.set(0);  
        
    }
    /** This method will be called once per scheduler run. */
    @Override
    public void periodic() {
        
    }

}