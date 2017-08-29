package it.iit.iCub.testTools;

import java.io.File;
import java.io.IOException;
import java.time.LocalDate;
import java.util.ArrayList;
import org.junit.After;
import org.junit.Before;
import org.junit.Rule;
import org.junit.rules.TestName;

import it.iit.iCub.IcubRobotModel;
import us.ihmc.avatar.DRCObstacleCourseStartingLocation;
import us.ihmc.avatar.DRCStartingLocation;
import us.ihmc.avatar.initialSetup.OffsetAndYawRobotInitialSetup;
import us.ihmc.avatar.testTools.DRCSimulationTestHelper;
import us.ihmc.commons.PrintTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.simulationConstructionSetTools.util.environments.CommonAvatarEnvironmentInterface;
import us.ihmc.simulationConstructionSetTools.util.environments.FlatGroundEnvironment;
import us.ihmc.simulationconstructionset.DataFileWriter;
import us.ihmc.simulationconstructionset.OneDegreeOfFreedomJoint;
import us.ihmc.simulationconstructionset.PinJoint;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.tools.MemoryTools;
import us.ihmc.tools.thread.ThreadTools;
import us.ihmc.yoVariables.variable.YoVariable;

public class ICubTest
{
   private static final boolean exportJointDataWhenDebugging = false;

   protected static final DRCStartingLocation startingLocation = DRCObstacleCourseStartingLocation.DEFAULT;
   protected static final IcubRobotModel robotModel = new IcubRobotModel();

   protected static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromEnvironmentVariables();
   static
   {
      /**
       * Avoid using state estimation for now until we have figured out the IMU:
       */
      simulationTestingParameters.setUsePefectSensors(true);

      /**
       * To avoid the simulation closing when the unit test is over for debugging uncomment:
       * (But do not commit this or the unit tests will hang on the test server!)
       */
//      simulationTestingParameters.setKeepSCSUp(true);
   }

   protected DRCSimulationTestHelper drcSimulationTestHelper;

   public void exportJointData()
   {
      SimulationConstructionSet scs = drcSimulationTestHelper.getSimulationConstructionSet();

      Robot robot = null;
      for (Robot scsRobot : scs.getRobots())
      {
         if (robotModel.getSimpleRobotName().equals(scsRobot.getName()))
         {
            robot = scsRobot;
            break;
         }
      }

      ArrayList<YoVariable<?>> vars = new ArrayList<>();
      ArrayList<OneDegreeOfFreedomJoint> joints = new ArrayList<>();
      robot.getAllOneDegreeOfFreedomJoints(joints);

      for (OneDegreeOfFreedomJoint joint : joints)
      {
         if (joint instanceof PinJoint)
         {
            PinJoint pinJoint = (PinJoint) joint;
            vars.add(pinJoint.getQYoVariable());
            vars.add(pinJoint.getQDYoVariable());
            vars.add(pinJoint.getTauYoVariable());
         }
      }
      vars.add(robot.getYoTime());

      String date = LocalDate.now().toString();
      String testName = name.getMethodName();
      File file = new File("Exports/" + robotModel.getSimpleRobotName() + "_" + testName + "_" + date + ".mat");

      try
      {
         PrintTools.info("Saving to file: " + file.getAbsolutePath());
         file.getParentFile().mkdirs();
         file.createNewFile();
         DataFileWriter dataWriter = new DataFileWriter(file);
         dataWriter.writeMatlabBinaryData(scs.getDT() * scs.getRecordFreq(), scs.getDataBuffer(), vars);
      }
      catch (IOException e)
      {
         e.printStackTrace();
      }
   }

   @Rule
   public TestName name = new TestName();

   @Before
   public void setupTest() throws SimulationExceededMaximumTimeException
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before test.");

      drcSimulationTestHelper = new DRCSimulationTestHelper(getEnvironment(), name.getMethodName(), startingLocation, simulationTestingParameters, robotModel);
      OffsetAndYawRobotInitialSetup startingLocationOffset = startingLocation.getStartingLocationOffset();
      Point3D cameraFocus = new Point3D(startingLocationOffset.getAdditionalOffset());
      cameraFocus.addZ(0.4);
      RigidBodyTransform transform = new RigidBodyTransform();
      transform.setRotationYawAndZeroTranslation(startingLocationOffset.getYaw());
      Point3D cameraPosition = new Point3D(5.0, 3.0, 1.0);
      transform.transform(cameraPosition);
      drcSimulationTestHelper.setupCameraForUnitTest(cameraFocus, cameraPosition);
      ThreadTools.sleep(1000);
   }

   protected CommonAvatarEnvironmentInterface getEnvironment()
   {
      return new FlatGroundEnvironment();
   }

   @After
   public void finishTest()
   {
      if (simulationTestingParameters.getKeepSCSUp())
      {
         if (exportJointDataWhenDebugging)
         {
            exportJointData();
         }
         ThreadTools.sleepForever();
      }

      // Do this here in case a test fails. That way the memory will be recycled.
      if (drcSimulationTestHelper != null)
      {
         drcSimulationTestHelper.destroySimulation();
         drcSimulationTestHelper = null;
      }

      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
   }
}
