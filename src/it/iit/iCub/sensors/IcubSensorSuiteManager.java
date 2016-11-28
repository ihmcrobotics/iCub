package it.iit.iCub.sensors;

import java.io.IOException;
import java.net.URI;

import us.ihmc.avatar.networkProcessor.lidarScanPublisher.LidarScanPublisher;
import us.ihmc.avatar.sensors.DRCSensorSuiteManager;
import us.ihmc.communication.net.ObjectCommunicator;
import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.communication.util.NetworkPorts;
import us.ihmc.humanoidRobotics.kryo.IHMCCommunicationKryoNetClassList;
import us.ihmc.humanoidRobotics.kryo.PPSTimestampOffsetProvider;
import us.ihmc.ihmcPerception.camera.CameraDataReceiver;
import us.ihmc.ihmcPerception.camera.SCSCameraDataReceiver;
import us.ihmc.robotModels.FullHumanoidRobotModelFactory;
import us.ihmc.sensorProcessing.communication.packets.dataobjects.RobotConfigurationData;
import us.ihmc.sensorProcessing.communication.producers.RobotConfigurationDataBuffer;
import us.ihmc.sensorProcessing.parameters.DRCRobotCameraParameters;
import us.ihmc.sensorProcessing.parameters.DRCRobotLidarParameters;
import us.ihmc.sensorProcessing.parameters.DRCRobotSensorInformation;
import us.ihmc.wholeBodyController.DRCRobotJointMap;

public class IcubSensorSuiteManager implements DRCSensorSuiteManager
{
   private final PacketCommunicator sensorSuitePacketCommunicator = PacketCommunicator.createIntraprocessPacketCommunicator(NetworkPorts.SENSOR_MANAGER,
         new IHMCCommunicationKryoNetClassList());

   private final RobotConfigurationDataBuffer robotConfigurationDataBuffer = new RobotConfigurationDataBuffer();
   private final PPSTimestampOffsetProvider ppsTimestampOffsetProvider;
   private final FullHumanoidRobotModelFactory modelFactory;
   private final DRCRobotSensorInformation sensorInformation;
   private final LidarScanPublisher lidarScanPublisher;

   public IcubSensorSuiteManager(FullHumanoidRobotModelFactory modelFactory, PPSTimestampOffsetProvider ppsTimestampOffsetProvider,
         DRCRobotSensorInformation sensorInformation, DRCRobotJointMap jointMap, boolean useSimulatedSensors)
   {
      this.modelFactory = modelFactory;
      this.sensorInformation = sensorInformation;
      this.ppsTimestampOffsetProvider = ppsTimestampOffsetProvider;
      DRCRobotLidarParameters lidarParameters = sensorInformation.getLidarParameters(0);
      String sensorName = lidarParameters.getSensorNameInSdf();
      lidarScanPublisher = new LidarScanPublisher(sensorName, modelFactory, sensorSuitePacketCommunicator);
      lidarScanPublisher.setPPSTimestampOffsetProvider(ppsTimestampOffsetProvider);
      lidarScanPublisher.setCollisionBoxProvider(null);
   }

   @Override
   public void initializeSimulatedSensors(ObjectCommunicator scsSensorsPacketCommunicator)
   {
      sensorSuitePacketCommunicator.attachListener(RobotConfigurationData.class, robotConfigurationDataBuffer);
      
      DRCRobotCameraParameters cameraParameters = sensorInformation.getCameraParameters(0);
      CameraDataReceiver cameraDataReceiver = new SCSCameraDataReceiver(cameraParameters.getRobotSide(), modelFactory, cameraParameters.getSensorNameInSdf(),
            robotConfigurationDataBuffer, scsSensorsPacketCommunicator, sensorSuitePacketCommunicator, ppsTimestampOffsetProvider);
      cameraDataReceiver.start();

      lidarScanPublisher.setScanFrameToLidarSensorFrame();
      lidarScanPublisher.receiveLidarFromSCS(scsSensorsPacketCommunicator);
      lidarScanPublisher.start();
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
}
