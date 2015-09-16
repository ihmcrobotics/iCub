package it.iit.iCub.sensors;

import java.io.IOException;
import java.net.URI;

import us.ihmc.SdfLoader.SDFFullRobotModelFactory;
import us.ihmc.communication.net.ObjectCommunicator;
import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.communication.util.NetworkPorts;
import us.ihmc.darpaRoboticsChallenge.networkProcessor.camera.CameraDataReceiver;
import us.ihmc.darpaRoboticsChallenge.networkProcessor.camera.SCSCameraDataReceiver;
import us.ihmc.darpaRoboticsChallenge.networkProcessor.depthData.PointCloudDataReceiver;
import us.ihmc.darpaRoboticsChallenge.networkProcessor.depthData.SCSPointCloudLidarReceiver;
import us.ihmc.darpaRoboticsChallenge.sensors.DRCSensorSuiteManager;
import us.ihmc.humanoidRobotics.kryo.IHMCCommunicationKryoNetClassList;
import us.ihmc.humanoidRobotics.kryo.PPSTimestampOffsetProvider;
import us.ihmc.sensorProcessing.communication.packets.dataobjects.RobotConfigurationData;
import us.ihmc.sensorProcessing.communication.producers.RobotConfigurationDataBuffer;
import us.ihmc.sensorProcessing.parameters.DRCRobotSensorInformation;
import us.ihmc.wholeBodyController.DRCRobotJointMap;

public class IcubSensorSuiteManager implements DRCSensorSuiteManager
{
   private final PacketCommunicator sensorSuitePacketCommunicator = PacketCommunicator.createIntraprocessPacketCommunicator(NetworkPorts.SENSOR_MANAGER,
         new IHMCCommunicationKryoNetClassList());

   private final RobotConfigurationDataBuffer robotConfigurationDataBuffer = new RobotConfigurationDataBuffer();
   private final PPSTimestampOffsetProvider ppsTimestampOffsetProvider;
   private final PointCloudDataReceiver pointCloudDataReceiver;
   private final SDFFullRobotModelFactory modelFactory;
   private final DRCRobotSensorInformation sensorInformation;
   
   public IcubSensorSuiteManager(SDFFullRobotModelFactory modelFactory, PPSTimestampOffsetProvider ppsTimestampOffsetProvider, DRCRobotSensorInformation sensorInformation,
         DRCRobotJointMap jointMap, boolean useSimulatedSensors)
   {
      this.modelFactory = modelFactory;
      this.sensorInformation = sensorInformation;
      this.ppsTimestampOffsetProvider = ppsTimestampOffsetProvider;
      this.pointCloudDataReceiver = new PointCloudDataReceiver(modelFactory, null, ppsTimestampOffsetProvider, jointMap, robotConfigurationDataBuffer,
            sensorSuitePacketCommunicator);
   }

   @Override
   public void initializeSimulatedSensors(ObjectCommunicator scsSensorsPacketCommunicator)
   {
      sensorSuitePacketCommunicator.attachListener(RobotConfigurationData.class, robotConfigurationDataBuffer);
      CameraDataReceiver cameraDataReceiver = new SCSCameraDataReceiver(sensorInformation.getCameraParameters(0).getRobotSide(), modelFactory, sensorInformation.getCameraParameters(0).getSensorNameInSdf(), robotConfigurationDataBuffer, scsSensorsPacketCommunicator, sensorSuitePacketCommunicator, ppsTimestampOffsetProvider);
      new SCSPointCloudLidarReceiver(sensorInformation.getLidarParameters(0).getSensorNameInSdf(), scsSensorsPacketCommunicator, pointCloudDataReceiver);
      cameraDataReceiver.start();
      pointCloudDataReceiver.start();
   }

   @Override
   public void initializePhysicalSensors(URI sensorURI)
   {

   }

   @Override
   public void connect() throws IOException
   {
      sensorSuitePacketCommunicator.connect();
   }


   //   @Override
   //   public PacketCommunicator createSensorModule(URI sensorURI)
   //   {
   //      KryoLocalPacketCommunicator sensorSuitePacketCommunicator = new KryoLocalPacketCommunicator(new IHMCCommunicationKryoNetClassList(),PacketDestination.SENSOR_MANAGER.ordinal(), "STEPPR_SENSOR_MANAGER");
   //      if(useSimulatedSensors)
   //      {
   //         initializeSimulatedSensors(sensorSuitePacketCommunicator);
   //      } else {
   //         initializePhysicalSensors(sensorSuitePacketCommunicator, sensorURI);
   //      }
   //      
   //      return sensorSuitePacketCommunicator;
   //   }

}
