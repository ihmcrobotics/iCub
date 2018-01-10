package it.iit.iCub.parameters;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import us.ihmc.commonWalkingControlModules.configurations.HighLevelControllerParameters;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.WholeBodySetpointParameters;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName;
import us.ihmc.sensorProcessing.outputData.JointDesiredControlMode;
import us.ihmc.sensorProcessing.outputData.JointDesiredBehavior;
import us.ihmc.sensorProcessing.outputData.JointDesiredBehaviorReadOnly;
import us.ihmc.sensorProcessing.outputData.JointDesiredControlMode;
import us.ihmc.commonWalkingControlModules.configurations.GroupParameter;

public class IcubHighLevelControllerParameters implements HighLevelControllerParameters
{
   private final IcubJointMap jointMap;
   private boolean runningOnRealRobot;

   public IcubHighLevelControllerParameters(boolean runningOnRealRobot, IcubJointMap jointMap)
   {
      this.runningOnRealRobot = runningOnRealRobot;
      this.jointMap = jointMap;
   }

   @Override
   public WholeBodySetpointParameters getStandPrepParameters()
   {
      return null;
   }

//   @Override
//   public JointDesiredControlMode getJointDesiredControlMode(String joint, HighLevelControllerName state)
//   {
//      return JointDesiredControlMode.EFFORT;
//   }
//
//   @Override
//   public double getDesiredJointStiffness(String joint, HighLevelControllerName state)
//   {
//      return 0.0;
//   }
//
//   @Override
//   public double getDesiredJointDamping(String joint, HighLevelControllerName state)
//   {
//      return 0.0;
//   }

   private List<GroupParameter<JointDesiredBehaviorReadOnly>> getDesiredJointBehaviorForDoNothing()
   {
      List<String> allJoints = Arrays.asList(jointMap.getOrderedJointNames());
      JointDesiredBehavior allJointBehaviors = new JointDesiredBehavior(JointDesiredControlMode.EFFORT);

      List<GroupParameter<JointDesiredBehaviorReadOnly>> behaviors = new ArrayList<>();
      behaviors.add(new GroupParameter<>("", allJointBehaviors, allJoints));
      return behaviors;
   }

   @Override
   public List<GroupParameter<JointDesiredBehaviorReadOnly>> getDesiredJointBehaviors(HighLevelControllerName state)
   {
      switch (state)
      {
         case WALKING:
            return getDesiredJointBehaviorForDoNothing();
         case DO_NOTHING_BEHAVIOR:
            return getDesiredJointBehaviorForDoNothing();
         default:
            throw new RuntimeException("Implement a desired joint behavior for the high level state " + state);
      }
   }

   @Override
   public HighLevelControllerName getDefaultInitialControllerState()
   {
      return HighLevelControllerName.WALKING;
   }

   @Override
   public HighLevelControllerName getFallbackControllerState()
   {
      return HighLevelControllerName.DO_NOTHING_BEHAVIOR;
   }

   @Override
   public boolean automaticallyTransitionToWalkingWhenReady()
   {
      return false;
   }

   @Override
   public double getTimeToMoveInStandPrep()
   {
      return 0.0;
   }

   @Override
   public double getMinimumTimeInStandReady()
   {
      return 0.0;
   }

   @Override
   public double getTimeInStandTransition()
   {
      return 0.0;
   }

   @Override
   public double getCalibrationDuration()
   {
      return 0.0;
   }
}
