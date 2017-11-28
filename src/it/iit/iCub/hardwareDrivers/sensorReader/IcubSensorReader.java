package it.iit.iCub.hardwareDrivers.sensorReader;

import gnu.trove.map.TIntObjectMap;
import it.iit.iCub.hardwareDrivers.yarp.IcubUDPRobotFeedbackReceiver;
import it.iit.iCub.messages.it.iit.yarp.JointState;
import it.iit.iCub.messages.it.iit.yarp.RobotFeedback;
import us.ihmc.commons.Conversions;
import us.ihmc.commons.PrintTools;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.sensorProcessing.communication.packets.dataobjects.AuxiliaryRobotData;
import us.ihmc.sensorProcessing.parameters.DRCRobotSensorInformation;
import us.ihmc.sensorProcessing.sensorProcessors.SensorOutputMapReadOnly;
import us.ihmc.sensorProcessing.sensorProcessors.SensorProcessing;
import us.ihmc.sensorProcessing.sensorProcessors.SensorRawOutputMapReadOnly;
import us.ihmc.sensorProcessing.sensors.RawJointSensorDataHolder;
import us.ihmc.sensorProcessing.sensors.RawJointSensorDataHolderMap;
import us.ihmc.sensorProcessing.simulatedSensors.SensorReader;
import us.ihmc.sensorProcessing.simulatedSensors.StateEstimatorSensorDefinitions;
import us.ihmc.sensorProcessing.stateEstimation.StateEstimatorParameters;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoLong;

import java.io.IOException;

/**
 * @author Doug Stephen <a href="mailto:dstephen@ihmc.us">(dstephen@ihmc.us)</a>
 */
public class IcubSensorReader implements SensorReader
{
   private final RobotFeedback robotFeedback;
   private final IcubUDPRobotFeedbackReceiver feedbackReceiver;

   private final SensorProcessing sensorProcessing;

   private final YoLong localDt;
   private final TIntObjectMap<OneDoFJoint> mapping;
   private final RawJointSensorDataHolderMap rawJointSensorDataHolderMap;

   public IcubSensorReader(RobotFeedback robotFeedback, TIntObjectMap<OneDoFJoint> mapping, StateEstimatorSensorDefinitions stateEstimatorSensorDefinitions,
                           RawJointSensorDataHolderMap rawJointSensorDataHolderMap, DRCRobotSensorInformation sensorInformation, StateEstimatorParameters stateEstimatorParameters,
                           YoVariableRegistry sensorReaderRegistry)
   {
      this.robotFeedback = robotFeedback;

      this.feedbackReceiver = new IcubUDPRobotFeedbackReceiver(IcubUDPRobotFeedbackReceiver.DEFAULT_YARP_FEEDBACK_IP,
                                                               IcubUDPRobotFeedbackReceiver.YARP_ROBOT_FEEDBACK_PORT, robotFeedback);
      this.mapping = mapping;

      this.rawJointSensorDataHolderMap = rawJointSensorDataHolderMap;

      this.sensorProcessing = new SensorProcessing(stateEstimatorSensorDefinitions, stateEstimatorParameters, sensorReaderRegistry);

      localDt = new YoLong("localDt", sensorReaderRegistry);
      localDt.set(-1);
   }

   public void connect() throws IOException
   {
      feedbackReceiver.connect();
   }

   public void disconnect()
   {
      feedbackReceiver.disconnect();
   }

   @Override
   public void read()
   {
      long receive = feedbackReceiver.receive();

      if (receive != -1)
      {
         if (localDt.getLongValue() == -1)
         {
            localDt.set(System.nanoTime());
         }
         else
         {
            localDt.set(System.nanoTime() - localDt.getLongValue());
         }

         for (int i = 0; i < robotFeedback.getJointStates().size(); i++)
         {
            JointState jointState = robotFeedback.getJointStates().get(i);
            OneDoFJoint oneDoFJoint = mapping.get(i);

            oneDoFJoint.setQ(jointState.getQ());
            oneDoFJoint.setQd(jointState.getQd());
            oneDoFJoint.setTau(jointState.getTau());

//            RawJointSensorDataHolder rawJointSensorDataHolder = rawJointSensorDataHolderMap.get(oneDoFJoint);
//
//            rawJointSensorDataHolder.setQ_raw(jointState.getQ());
//            rawJointSensorDataHolder.setQd_raw(jointState.getQd());
//
//            sensorProcessing.setJointPositionSensorValue(oneDoFJoint, jointState.getQ());
//            sensorProcessing.setJointVelocitySensorValue(oneDoFJoint, jointState.getQd());
//            sensorProcessing.setJointTauSensorValue(oneDoFJoint, jointState.getTau());
         }

         sensorProcessing.startComputation(receive, receive, -1);
      }
      else
      {
         PrintTools.error("Could not receive or deserialize feedback from YARP!");
      }
   }

   @Override
   public SensorOutputMapReadOnly getSensorOutputMapReadOnly()
   {
      return sensorProcessing;
   }

   @Override
   public SensorRawOutputMapReadOnly getSensorRawOutputMapReadOnly()
   {
      return sensorProcessing;
   }

   @Override
   public AuxiliaryRobotData newAuxiliaryRobotDataInstance()
   {
      return null;
   }
}
