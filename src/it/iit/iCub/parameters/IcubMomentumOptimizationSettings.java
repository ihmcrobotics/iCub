package it.iit.iCub.parameters;

import gnu.trove.map.hash.TObjectDoubleHashMap;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.MomentumOptimizationSettings;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.partNames.ArmJointName;
import us.ihmc.robotics.partNames.NeckJointName;
import us.ihmc.robotics.partNames.SpineJointName;
import us.ihmc.robotics.robotSide.RobotSide;

public class IcubMomentumOptimizationSettings extends MomentumOptimizationSettings
{
   private final Vector3D linearMomentumWeight = new Vector3D(0.05, 0.05, 0.01);
   private final Vector3D highLinearMomentumWeightForRecovery = new Vector3D(0.5, 0.5, 0.05);
   private final Vector3D angularMomentumWeight = new Vector3D(0.0, 0.0, 0.0);

   private final Vector3D defaultAngularFootWeight = new Vector3D(0.5, 0.5, 0.5);
   private final Vector3D defaultLinearFootWeight = new Vector3D(30.0, 30.0, 30.0);
   private final Vector3D highAngularFootWeight = new Vector3D(5.0, 5.0, 5.0);
   private final Vector3D highLinearFootWeight = new Vector3D(50.0, 50.0, 50.0);

   private final Vector3D chestAngularWeight = new Vector3D(15.0, 10.0, 5.0);
   private final Vector3D pelvisAngularWeight = new Vector3D(5.0, 5.0, 5.0);

   private final int nBasisVectorsPerContactPoint = 4;
   private final int nContactPointsPerContactableBody = 4;
   private final int nContactableBodies = 2;

   private final double jointAccelerationWeight = 0.005;
   private final double jointJerkWeight = 0.1;
   private final double rhoWeight = 0.000002;
   private final double rhoMin = 0.5;
   private final double rhoRateDefaultWeight = 0.0004;
   private final double rhoRateHighWeight = 0.01;
   private final Vector2D copWeight = new Vector2D(100.0, 200.0);
   private final Vector2D copRateDefaultWeight = new Vector2D(20000.0, 20000.0);
   private final Vector2D copRateHighWeight = new Vector2D(2500000.0, 10000000.0);
   private final double headTaskspaceWeight = 1.0;
   private final double headUserModeWeight = 1.0;
   private final double handUserModeWeight = 50.0;
   private final Vector3D handAngularTaskspaceWeight = new Vector3D(1.0, 1.0, 1.0);
   private final Vector3D handLinearTaskspaceWeight = new Vector3D(1.0, 1.0, 1.0);

   private final double neckJointspaceWeight = 1.0;
   private final double spineJointspaceWeight = 1.0;
   private final double armJointspaceWeight = 1.0;
   private final TObjectDoubleHashMap<String> jointspaceWeights = new TObjectDoubleHashMap<>();

   public IcubMomentumOptimizationSettings(IcubJointMap jointMap)
   {
      for (SpineJointName jointName : jointMap.getSpineJointNames())
         jointspaceWeights.put(jointMap.getSpineJointName(jointName), spineJointspaceWeight);

      for (ArmJointName jointName : jointMap.getArmJointNames())
         for (RobotSide robotSide : RobotSide.values)
            jointspaceWeights.put(jointMap.getArmJointName(robotSide, jointName), armJointspaceWeight);

      for (NeckJointName jointName : jointMap.getNeckJointNames())
         jointspaceWeights.put(jointMap.getNeckJointName(jointName), neckJointspaceWeight);
   }

   /** @inheritDoc */
   @Override
   public Vector3D getLinearMomentumWeight()
   {
      return linearMomentumWeight;
   }

   /** @inheritDoc */
   @Override
   public Vector3D getHighLinearMomentumWeightForRecovery()
   {
      return highLinearMomentumWeightForRecovery;
   }

   /** @inheritDoc */
   @Override
   public Vector3D getAngularMomentumWeight()
   {
      return angularMomentumWeight;
   }

   /** @inheritDoc */
   @Override
   public double getJointAccelerationWeight()
   {
      return jointAccelerationWeight;
   }

   /** @inheritDoc */
   @Override
   public double getJointJerkWeight()
   {
      return jointJerkWeight;
   }

   /** @inheritDoc */
   @Override
   public double getRhoWeight()
   {
      return rhoWeight;
   }

   /** @inheritDoc */
   @Override
   public double getRhoMin()
   {
      return rhoMin;
   }

   /** @inheritDoc */
   @Override
   public double getRhoRateDefaultWeight()
   {
      return rhoRateDefaultWeight;
   }

   /** @inheritDoc */
   @Override
   public double getRhoRateHighWeight()
   {
      return rhoRateHighWeight;
   }

   /** @inheritDoc */
   @Override
   public Vector2D getCoPWeight()
   {
      return copWeight;
   }

   /** @inheritDoc */
   @Override
   public Vector2D getCoPRateDefaultWeight()
   {
      return copRateDefaultWeight;
   }

   /** @inheritDoc */
   @Override
   public Vector2D getCoPRateHighWeight()
   {
      return copRateHighWeight;
   }

   /** @inheritDoc */
   @Override
   public double getHeadUserModeWeight()
   {
      return headUserModeWeight;
   }

   /** @inheritDoc */
   @Override
   public double getHeadJointspaceWeight()
   {
      return neckJointspaceWeight;
   }

   /** @inheritDoc */
   @Override
   public double getHeadTaskspaceWeight()
   {
      return headTaskspaceWeight;
   }

   /** @inheritDoc */
   @Override
   public Vector3D getChestAngularWeight()
   {
      return chestAngularWeight;
   }

   /** @inheritDoc */
   @Override
   public Vector3D getPelvisAngularWeight()
   {
      return pelvisAngularWeight;
   }

   /** @inheritDoc */
   @Override
   public Vector3D getDefaultLinearFootWeight()
   {
      return defaultLinearFootWeight;
   }

   /** @inheritDoc */
   @Override
   public Vector3D getDefaultAngularFootWeight()
   {
      return defaultAngularFootWeight;
   }

   /** @inheritDoc */
   @Override
   public Vector3D getHighLinearFootWeight()
   {
      return highLinearFootWeight;
   }

   /** @inheritDoc */
   @Override
   public Vector3D getHighAngularFootWeight()
   {
      return highAngularFootWeight;
   }

   /** @inheritDoc */
   @Override
   public double getHandUserModeWeight()
   {
      return handUserModeWeight;
   }

   /** @inheritDoc */
   @Override
   public double getHandJointspaceWeight()
   {
      return armJointspaceWeight;
   }

   /** @inheritDoc */
   @Override
   public Vector3D getHandAngularTaskspaceWeight()
   {
      return handAngularTaskspaceWeight;
   }

   /** @inheritDoc */
   @Override
   public Vector3D getHandLinearTaskspaceWeight()
   {
      return handLinearTaskspaceWeight;
   }

   /** @inheritDoc */
   @Override
   public int getNumberOfBasisVectorsPerContactPoint()
   {
      return nBasisVectorsPerContactPoint;
   }

   /** @inheritDoc */
   @Override
   public int getNumberOfContactPointsPerContactableBody()
   {
      return nContactPointsPerContactableBody;
   }

   /** @inheritDoc */
   @Override
   public int getNumberOfContactableBodies()
   {
      return nContactableBodies;
   }

   /** @inheritDoc */
   @Override
   public int getRhoSize()
   {
      return nContactableBodies * nContactPointsPerContactableBody * nBasisVectorsPerContactPoint;
   }

   /** @inheritDoc */
   @Override
   public TObjectDoubleHashMap<String> getJointspaceWeights()
   {
      return jointspaceWeights;
   }
}
