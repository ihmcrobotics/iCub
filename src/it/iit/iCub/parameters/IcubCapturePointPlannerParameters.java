package it.iit.iCub.parameters;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.commonWalkingControlModules.configurations.ContinuousCMPICPPlannerParameters;
import us.ihmc.euclid.tuple2D.Vector2D;

/** {@inheritDoc} */
public class IcubCapturePointPlannerParameters extends ContinuousCMPICPPlannerParameters
{
   private final List<Vector2D> copOffsets;
   private final List<Vector2D> copForwardOffsetBounds;

   public IcubCapturePointPlannerParameters()
   {
      Vector2D entryOffset = new Vector2D(0.0, 0.01);
      Vector2D exitOffset = new Vector2D(0.0, 0.01);
      copOffsets = new ArrayList<>();
      copOffsets.add(entryOffset);
      copOffsets.add(exitOffset);

      Vector2D entryBounds = new Vector2D(-0.03, Double.POSITIVE_INFINITY);
      Vector2D exitBounds = new Vector2D(Double.NEGATIVE_INFINITY, 0.06);
      copForwardOffsetBounds = new ArrayList<>();
      copForwardOffsetBounds.add(entryBounds);
      copForwardOffsetBounds.add(exitBounds);
   }

   @Override
   public int getNumberOfCoPWayPointsPerFoot()
   {
      return 2;
   }

   @Override
   public List<Vector2D> getCoPOffsets()
   {
      return copOffsets;
   }

   @Override
   public List<Vector2D> getCoPForwardOffsetBounds()
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
}
