package it.iit.iCub.parameters;

import java.util.ArrayList;

import org.apache.commons.lang3.tuple.ImmutableTriple;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.sensorProcessing.parameters.DRCRobotCameraParameters;
import us.ihmc.sensorProcessing.parameters.DRCRobotLidarParameters;
import us.ihmc.sensorProcessing.parameters.DRCRobotPointCloudParameters;
import us.ihmc.sensorProcessing.parameters.DRCRobotSensorInformation;
import us.ihmc.sensorProcessing.parameters.DRCRobotSensorParameters;

public class IcubSensorInformation implements DRCRobotSensorInformation
{
   /**
    * Force Sensor Parameters
    */
   public static final String[] forceSensorNames = {"l_ankle_roll", "r_ankle_roll"};
   public static final SideDependentList<String> feetForceSensorNames = new SideDependentList<String>("l_ankle_roll", "r_ankle_roll");
   public static final SideDependentList<String> handForceSensorNames = new SideDependentList<String>();

   public static final SideDependentList<RigidBodyTransform> transformFromMeasurementToAnkleZUpFrames = new SideDependentList<>();
   static
   {
      for (RobotSide side : RobotSide.values)
      {
         RigidBodyTransform transform = new RigidBodyTransform();
         transform.setTranslation(new Vector3D(-0.0035, 0.0, 0.0685));
         transformFromMeasurementToAnkleZUpFrames.put(side, transform);
      }
   }

   /**
    * Camera Parameters
    */
   private final DRCRobotCameraParameters[] cameraParamaters = new DRCRobotCameraParameters[0];

   /**
    * Lidar Parameters
    */
   private final DRCRobotLidarParameters[] lidarParamaters = new DRCRobotLidarParameters[0];

   /**
    * Stereo Parameters
    */
   private final DRCRobotPointCloudParameters[] pointCloudParamaters = new DRCRobotPointCloudParameters[0];

   /**
    * IMU
    */
   private static final String bodyIMUSensor = "head_imu_sensor";
   private static final String[] imuSensorsToUse = {bodyIMUSensor};

   @Override
   public String[] getIMUSensorsToUseInStateEstimator()
   {
      return imuSensorsToUse;
   }

   @Override
   public String[] getForceSensorNames()
   {
      return forceSensorNames;
   }

   @Override
   public SideDependentList<String> getFeetForceSensorNames()
   {
      return feetForceSensorNames;
   }

   @Override
   public SideDependentList<String> getWristForceSensorNames()
   {
      return handForceSensorNames;
   }

   @Override
   public String getPrimaryBodyImu()
   {
      return bodyIMUSensor;
   }

   @Override
   public DRCRobotCameraParameters[] getCameraParameters()
   {
      return cameraParamaters;
   }

   @Override
   public DRCRobotLidarParameters[] getLidarParameters()
   {
      return lidarParamaters;
   }

   @Override
   public DRCRobotPointCloudParameters[] getPointCloudParameters()
   {
      return pointCloudParamaters;
   }

   @Override
   public DRCRobotCameraParameters getCameraParameters(int cameraId)
   {
      return null;
   }

   @Override
   public DRCRobotLidarParameters getLidarParameters(int lidarId)
   {
      return null;
   }

   @Override
   public DRCRobotPointCloudParameters getPointCloudParameters(int pointCloudSensorId)
   {
      return null;
   }

   @Override
   public ReferenceFrame getHeadIMUFrameWhenLevel()
   {
      return null;
   }

   @Override
   public String[] getSensorFramesToTrack()
   {
      ArrayList<String> sensorFramesToTrack = new ArrayList<String>();
      sensorFramesToTrack(cameraParamaters, sensorFramesToTrack);
      sensorFramesToTrack(lidarParamaters, sensorFramesToTrack);
      sensorFramesToTrack(pointCloudParamaters, sensorFramesToTrack);
      String[] sensorFramesToTrackAsPrimitive = new String[sensorFramesToTrack.size()];
      sensorFramesToTrack.toArray(sensorFramesToTrackAsPrimitive);
      return sensorFramesToTrackAsPrimitive;
   }

   private void sensorFramesToTrack(DRCRobotSensorParameters[] sensorParams, ArrayList<String> holder)
   {
      for (int i = 0; i < sensorParams.length; i++)
      {
         if (sensorParams[i].getPoseFrameForSdf() != null)
         {
            holder.add(sensorParams[i].getPoseFrameForSdf());
         }
      }
   }

   @Override
   public boolean setupROSLocationService()
   {
      return false;
   }

   @Override
   public boolean setupROSParameterSetters()
   {
      return false;
   }

   @Override
   public boolean isMultisenseHead()
   {
      return false;
   }

   @Override
   public SideDependentList<String> getFeetContactSensorNames()
   {
      return feetForceSensorNames;
   }

   @Override
   public ArrayList<ImmutableTriple<String, String, RigidBodyTransform>> getStaticTransformsForRos()
   {
      return null;
   }
}
