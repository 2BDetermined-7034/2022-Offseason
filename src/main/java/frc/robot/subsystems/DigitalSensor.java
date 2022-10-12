package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class DigitalSensor extends SubsystemBase {
    //private final AnalogInput m_sensor;
    //private final AnalogInput m_sensor2;
    private final DigitalInput m_topSensor; // Port 0
    private final DigitalInput m_botSensor; // Port 1
    public boolean override;

    public DigitalSensor() {
        override = false;
        //m_sensor = new AnalogInput(0);
        //m_sensor2 = new AnalogInput(1);
        m_topSensor = new DigitalInput(0);
        m_botSensor = new DigitalInput(1);
    }


    public void debug(){
        SmartDashboard.putBoolean("TopFull?", getTopSensor());
        SmartDashboard.putBoolean("BotFull?", getBotSensor());
        SmartDashboard.putBoolean("Is Indexer Full?", sensorBoolean1_2());
    }
    public void disable(){

    }


    /**
     *
     * @return Double - Sensor 0's average value
     */
    public boolean getTopSensor() {
        return !m_topSensor.get();
    }

    /**
     *
     * @return Double - Sensor 1's average value
     */
    public boolean getBotSensor() {
        return !m_botSensor.get();
    }

    /**
     *
     * @return Boolean / If the color sensors have something in front of them - True = There is something in front of the color sensor.
     */
    public boolean sensorBoolean1_2() {
        boolean sen1  = getTopSensor();
        boolean sen2 = getBotSensor();

        if(override) return false;
        return sen1 && sen2;
    }

    @Override
    public void periodic() {
        debug();
    }
}