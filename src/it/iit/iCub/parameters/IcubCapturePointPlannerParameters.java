package it.iit.iCub.parameters;

import java.util.EnumMap;

import us.ihmc.commonWalkingControlModules.configurations.ContinuousCMPICPPlannerParameters;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.commonWalkingControlModules.configurations.CoPPointName;

/** {@inheritDoc} */
public class IcubCapturePointPlannerParameters extends ContinuousCMPICPPlannerParameters
{
   private final boolean runningOnRealRobot;
   private final boolean useTwoCMPsPerSupport;

   private final CoPPointName exitCoPName = CoPPointName.TOE_COP;
   private final CoPPointName entryCoPName = CoPPointName.HEEL_COP;

   private EnumMap<CoPPointName, Vector2D> copOffsets;
   private EnumMap<CoPPointName, Vector2D> copForwardOffsetBounds;

   public IcubCapturePointPlannerParameters(boolean runningOnRealRobot)
   {
      this.runningOnRealRobot = runningOnRealRobot;
      useTwoCMPsPerSupport = true;
   }

   @Override
   public int getNumberOfCoPWayPointsPerFoot()
   {
      if (useTwoCMPsPerSupport)
         return 2;
      else
         return 1;
   }

   /**{@inheritDoc} */
   @Override
   public CoPPointName getExitCoPName()
   {
      return exitCoPName;
   }

   /**{@inheritDoc} */
   @Override
   public CoPPointName getEntryCoPName()
   {
      return entryCoPName;
   }

   /**{@inheritDoc} */
   @Override
   public EnumMap<CoPPointName, Vector2D> getCoPOffsetsInFootFrame()
   {
      if (copOffsets != null)
         return copOffsets;

      Vector2D entryOffset, exitOffset;
      if (runningOnRealRobot)
         entryOffset = new Vector2D(0.01, 0.01);
      else
         entryOffset = new Vector2D(0.0, 0.01);

      exitOffset = new Vector2D(0.0, 0.01);

      copOffsets = new EnumMap<>(CoPPointName.class);
      copOffsets.put(entryCoPName, entryOffset);
      copOffsets.put(exitCoPName, exitOffset);

      return copOffsets;
   }

   /** {@inheritDoc} */
   @Override
   public EnumMap<CoPPointName, Vector2D> getCoPForwardOffsetBoundsInFoot()
   {
      if (copForwardOffsetBounds != null)
         return copForwardOffsetBounds;

      Vector2D entryBounds = new Vector2D(-0.03, Double.POSITIVE_INFINITY);
      Vector2D exitBounds = new Vector2D(Double.NEGATIVE_INFINITY, 0.06);

      copForwardOffsetBounds = new EnumMap<>(CoPPointName.class);
      copForwardOffsetBounds.put(entryCoPName, entryBounds);
      copForwardOffsetBounds.put(exitCoPName, exitBounds);

      return copForwardOffsetBounds;
   }

   /**{@inheritDoc} */
   @Override
   public double getCoPSafeDistanceAwayFromSupportEdges()
   {
      return 0.005;
   }

   /**{@inheritDoc} */
   @Override
   public double getMinTimeToSpendOnExitCoPInSingleSupport()
   {
      return 0.0;
   }

   /**{@inheritDoc} */
   @Override
   public double getSwingSplitFraction()
   {
      return 0.35;
   }

   /**{@inheritDoc} */
   @Override
   public double getTransferSplitFraction()
   {
      return 0.5;
   }

   /** {@inheritDoc} */
   @Override
   public double getVelocityDecayDurationWhenDone()
   {
      return 0.5;
   }
}
