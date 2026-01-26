package frc.robot.util.motorUtil;

public class RelEncoderFollower extends RelEncoderSparkMax{
  RelEncoderSparkMax m_leadMotor;
  public RelEncoderFollower(MotorConfig motorConfig, RelEncoderSparkMax leadMotor){
    super(motorConfig);
    m_leadMotor = leadMotor;
  }

  @Override
  public void configureMotor(){
    super.configureMotor();
    super.getSparMaxConfig().follow(m_leadMotor.getSparkMax());
  }
}
