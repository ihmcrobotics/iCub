package it.iit.iCub.parameters;

import java.util.HashMap;
import java.util.Map;
import java.util.List;
import java.util.ArrayList;
import java.util.Collections;

import gnu.trove.map.hash.TObjectDoubleHashMap;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.MomentumOptimizationSettings;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.partNames.ArmJointName;
import us.ihmc.robotics.partNames.NeckJointName;
import us.ihmc.robotics.partNames.SpineJointName;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.commonWalkingControlModules.configurations.GroupParameter;
import us.ihmc.wholeBodyController.DRCRobotJointMap;

public class IcubMomentumOptimizationSettings extends MomentumOptimizationSettings
{
   // defaults for unscaled model:
   private static final double defaultRhoWeight = 0.00001;
   private static final double defaultRhoMin = 4.0;
   private static final double defaultRhoRateDefaultWeight = 0.002;
   private static final double defaultRhoRateHighWeight = 0.05;

   private final Vector3D linearMomentumWeight = new Vector3D(2.0, 2.0, 0.25);
   private final Vector3D highLinearMomentumWeightForRecovery = linearMomentumWeight;
   private final Vector3D angularMomentumWeight = new Vector3D(0.0, 0.0, 0.0);

   // swing and movie via waypoints
   private final Vector3D defaultAngularFootWeight = new Vector3D(2.0, 0.5, 2.0);
   private final Vector3D defaultLinearFootWeight = new Vector3D(200.0, 200.0, 200.0);
   // support and on toes
   private final Vector3D highAngularFootWeight = new Vector3D(1.0, 1.0, 1.0);
   private final Vector3D highLinearFootWeight = new Vector3D(50.0, 50.0, 50.0);

   private final Vector3D pelvisAngularWeight = new Vector3D(10.0, 10.0, 1.0);
   private final Vector3D pelvisLinearWeight = new Vector3D(5.0, 5.0, 50.0);

   private final int nBasisVectorsPerContactPoint = 4;
   private final int nContactPointsPerContactableBody = 4;
   private final int nContactableBodies;

   private final double jointAccelerationWeight = 0.001;
   private final double jointJerkWeight = 0.01;
   private final double rhoWeight = 0.001;
   private final double rhoMin = 1.0 / (nBasisVectorsPerContactPoint * nContactPointsPerContactableBody);
   private final double rhoRateDefaultWeight = 0.01;
   private final double rhoRateHighWeight = rhoRateDefaultWeight;
   private final Vector2D copWeight = new Vector2D(1000.0, 1000.0);
   private final Vector2D copRateDefaultWeight = new Vector2D(0.0, 0.0);
   private final Vector2D copRateHighWeight = new Vector2D(0.0, 0.0);

   private final double neckJointspaceWeight = 10.0;
   private final double armJointspaceWeight = 1.0;
   private final double spineJointspaceWeightYaw = 15.0;
   private final double spineJointspaceWeightPitch = 15.0;
   private final double spineJointspaceWeightRoll = 15.0;
   private final List<GroupParameter<Double>> jointspaceWeights = new ArrayList<>();

   private final double neckUserModeWeight = 1.0;
   private final double spineUserModeWeight = 50.0;
   private final double armUserModeWeight = 50.0;
   private final List<GroupParameter<Double>> userModeWeights = new ArrayList<>();

   private final Vector3D headAngularWeight = new Vector3D(1.0, 1.0, 1.0);
   private final Vector3D chestAngularWeight = new Vector3D(15.0, 10.0, 5.0);
   private final Vector3D handAngularWeight = new Vector3D(1.0, 1.0, 1.0);
   private final List<GroupParameter<Vector3DReadOnly>> taskspaceAngularWeights = new ArrayList<>();

   private final Vector3D handLinearWeight = new Vector3D(5.0, 5.0, 5.0);
   private final List<GroupParameter<Vector3DReadOnly>> taskspaceLinearWeights = new ArrayList<>();

   public IcubMomentumOptimizationSettings(IcubJointMap jointMap, int numberOfContactableBodies)
   {
      double scale = Math.pow(jointMap.getModelScale(), jointMap.getMassScalePower());

//      rhoWeight = defaultRhoWeight / scale;
//      rhoMin = defaultRhoMin * scale;
//      rhoRateDefaultWeight = defaultRhoRateDefaultWeight / (scale * scale);
//      rhoRateHighWeight = defaultRhoRateHighWeight / (scale * scale);

      linearMomentumWeight.scale(1.0 / scale);
      highLinearMomentumWeightForRecovery.scale(1.0 / scale);
      angularMomentumWeight.scale(1.0 / scale);

      userModeWeights.add(new GroupParameter<>("Spine", spineUserModeWeight, jointMap.getSpineJointNamesAsStrings()));
      configureBehavior(jointspaceWeights, jointMap, SpineJointName.SPINE_YAW, spineJointspaceWeightYaw);
      configureBehavior(jointspaceWeights, jointMap, SpineJointName.SPINE_PITCH, spineJointspaceWeightPitch);
      configureBehavior(jointspaceWeights, jointMap, SpineJointName.SPINE_ROLL, spineJointspaceWeightRoll);

      jointspaceWeights.add(new GroupParameter<>("Arms", armJointspaceWeight, jointMap.getArmJointNamesAsStrings()));
      userModeWeights.add(new GroupParameter<>("Arms", armUserModeWeight, jointMap.getArmJointNamesAsStrings()));

      jointspaceWeights.add(new GroupParameter<>("Neck", neckJointspaceWeight, jointMap.getNeckJointNamesAsStrings()));
      userModeWeights.add(new GroupParameter<>("Neck", neckUserModeWeight, jointMap.getNeckJointNamesAsStrings()));

      taskspaceAngularWeights.add(new GroupParameter<>("Chest", chestAngularWeight, Collections.singletonList(jointMap.getChestName())));
      taskspaceAngularWeights.add(new GroupParameter<>("Head", headAngularWeight, Collections.singletonList(jointMap.getHeadName())));

      taskspaceAngularWeights.add(new GroupParameter<>("Pelvis", pelvisAngularWeight, Collections.singletonList(jointMap.getPelvisName())));
      taskspaceLinearWeights.add(new GroupParameter<>("Pelvis", pelvisLinearWeight, Collections.singletonList(jointMap.getPelvisName())));

      List<String> handNames = new ArrayList<>();
      for (RobotSide robotSide : RobotSide.values)
      {
         handNames.add(jointMap.getHandName(robotSide));
      }
      taskspaceAngularWeights.add(new GroupParameter<>("Hand", handAngularWeight, handNames));
      taskspaceLinearWeights.add(new GroupParameter<>("Hand", handLinearWeight, handNames));

      this.nContactableBodies = numberOfContactableBodies;
   }

   private static void configureBehavior(List<GroupParameter<Double>> behaviors, DRCRobotJointMap jointMap, SpineJointName jointName, double weight)
   {
      List<String> names = Collections.singletonList(jointMap.getSpineJointName(jointName));
      behaviors.add(new GroupParameter<>(jointName.toString(), new Double(weight), names));
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

//   /** @inheritDoc */
//   @Override
//   public double getHeadUserModeWeight()
//   {
//      return neckUserModeWeight;
//   }
//
//   /** @inheritDoc */
//   @Override
//   public double getHeadJointspaceWeight()
//   {
//      return neckJointspaceWeight;
//   }
//
//   /** @inheritDoc */
//   @Override
//   public Vector3D getHeadAngularWeight()
//   {
//      return headAngularWeight;
//   }
//
//   /** @inheritDoc */
//   @Override
//   public Vector3D getChestAngularWeight()
//   {
//      return chestAngularWeight;
//   }
//
//   /** @inheritDoc */
//   @Override
//   public Vector3D getPelvisAngularWeight()
//   {
//      return pelvisAngularWeight;
//   }

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

//   /** @inheritDoc */
//   @Override
//   public double getHandUserModeWeight()
//   {
//      return armUserModeWeight;
//   }
//
//   /** @inheritDoc */
//   @Override
//   public double getHandJointspaceWeight()
//   {
//      return armJointspaceWeight;
//   }
//
//   /** @inheritDoc */
//   @Override
//   public Vector3D getHandAngularTaskspaceWeight()
//   {
//      return handAngularWeight;
//   }
//
//   /** @inheritDoc */
//   @Override
//   public Vector3D getHandLinearTaskspaceWeight()
//   {
//      return handLinearWeight;
//   }

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
   public List<GroupParameter<Double>> getJointspaceWeights()
   {
      return jointspaceWeights;
   }

//   /** @inheritDoc */
//   @Override
//   public double getChestUserModeWeight()
//   {
//      return spineUserModeWeight;
//   }

   /** @inheritDoc */
   @Override
   public List<GroupParameter<Double>> getUserModeWeights()
   {
      return userModeWeights;
   }

   /** @inheritDoc */
   @Override
   public List<GroupParameter<Vector3DReadOnly>> getTaskspaceAngularWeights()
   {
      return taskspaceAngularWeights;
   }

   /** @inheritDoc */
   @Override
   public List<GroupParameter<Vector3DReadOnly>> getTaskspaceLinearWeights()
   {
      return taskspaceLinearWeights;
   }

//   @Override
//   public Vector3D getPelvisLinearWeight()
//   {
//      return pelvisLinearWeight;
//   }
}
