package it.iit.iCub.parameters;

import java.util.EnumMap;

import us.ihmc.commonWalkingControlModules.configurations.CoPPointName;
import us.ihmc.commonWalkingControlModules.configurations.ContinuousCMPICPPlannerParameters;
import us.ihmc.euclid.tuple2D.Vector2D;

/** {@inheritDoc} */
public class IcubCapturePointPlannerParameters extends ContinuousCMPICPPlannerParameters
{
   private final EnumMap<CoPPointName, Vector2D> copOffsets;
   private final EnumMap<CoPPointName, Vector2D> copForwardOffsetBounds;

   public IcubCapturePointPlannerParameters()
   {
      Vector2D entryOffset = new Vector2D(0.0, 0.01);
      Vector2D exitOffset = new Vector2D(0.0, 0.01);
      copOffsets = new EnumMap<>(CoPPointName.class);
      copOffsets.put(entryCoPName, entryOffset);
      copOffsets.put(exitCoPName, exitOffset);

      Vector2D entryBounds = new Vector2D(-0.03, Double.POSITIVE_INFINITY);
      Vector2D exitBounds = new Vector2D(Double.NEGATIVE_INFINITY, 0.06);
      copForwardOffsetBounds = new EnumMap<>(CoPPointName.class);
      copForwardOffsetBounds.put(entryCoPName, entryBounds);
      copForwardOffsetBounds.put(exitCoPName, exitBounds);
   }

   @Override
   public int getNumberOfCoPWayPointsPerFoot()
   {
      return 2;
   }

   @Override
   public EnumMap<CoPPointName, Vector2D> getCoPOffsetsInFootFrame()
   {
      return copOffsets;
   }

   @Override
   public EnumMap<CoPPointName, Vector2D> getCoPForwardOffsetBoundsInFoot()
   {
      return copForwardOffsetBounds;
   }

   @Override
   public double getCoPSafeDistanceAwayFromSupportEdges()
   {
      return 0.005;
   }

   @Override
   public double getMinTimeToSpendOnExitCoPInSingleSupport()
   {
      return 0.0;
   }

   @Override
   public double getSwingSplitFraction()
   {
      return 0.35;
   }

   @Override
   public double getTransferSplitFraction()
   {
      return 0.5;
   }

   @Override
   public CoPPointName getExitCoPName()
   {
      return exitCoPName;
   }

   @Override
   public CoPPointName getEntryCoPName()
   {
      return entryCoPName;
   }
}
