package it.iit.iCub.testTools;

import java.awt.Color;
import java.io.ByteArrayOutputStream;
import java.io.File;
import java.io.IOException;
import java.io.PrintStream;
import java.time.LocalDate;
import java.util.ArrayList;

import org.junit.After;
import org.junit.Assert;
import org.junit.Before;
import org.junit.Rule;
import org.junit.rules.TestName;

import it.iit.iCub.IcubRobotModel;
import it.iit.iCub.roughTerrain.ICubRampsTest;
import it.iit.iCub.roughTerrain.ICubWobblyFeetTest;
import us.ihmc.avatar.DRCObstacleCourseStartingLocation;
import us.ihmc.avatar.DRCStartingLocation;
import us.ihmc.avatar.initialSetup.OffsetAndYawRobotInitialSetup;
import us.ihmc.avatar.testTools.DRCSimulationTestHelper;
import us.ihmc.commons.PrintTools;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.geometry.BoundingBox3D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.YoAppearanceRGBColor;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.simulationConstructionSetTools.util.environments.CommonAvatarEnvironmentInterface;
import us.ihmc.simulationConstructionSetTools.util.environments.FlatGroundEnvironment;
import us.ihmc.simulationToolkit.controllers.PushRobotController;
import us.ihmc.simulationconstructionset.DataFileWriter;
import us.ihmc.simulationconstructionset.OneDegreeOfFreedomJoint;
import us.ihmc.simulationconstructionset.PinJoint;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.tools.MemoryTools;
import us.ihmc.yoVariables.variable.YoVariable;

public abstract class ICubTest
{
   private IcubRobotModel robotModel;
   private DRCSimulationTestHelper drcSimulationTestHelper;
   private PushRobotController pushRobotController;

   private static final boolean exportJointDataWhenDebugging = false;
   private static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromSystemProperties();
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

   /**
    * Overwrite if your test requires the joint limits on the robot to be removed. By default all
    * tests will assume the joint limits are active.
    */
   public boolean removeJointLimits()
   {
      return false;
   }

   /**
    * Overwrite if the initial camera position in the unit test should be at a different spot. The
    * initial camera focus will always be the robot.
    */
   public Point3D getCameraPosion()
   {
      return new Point3D(5.0, 3.0, 1.0);
   }

   /**
    * Overwrite if your unit test requires a different terrain. By default all unit tests use a flat
    * ground environment. To define a unit test specific terrain you can use the {@link TestingEnvironment}.
    * For an example of how to use it look at {@link ICubRampsTest.RampEnvironment}.
    */
   public CommonAvatarEnvironmentInterface getEnvironment()
   {
      return new FlatGroundEnvironment();
   }

   /**
    * Overwrite to add a check at the end of the unit test. If this method returns a bounding box the
    * pelvis of the robot after the test must be inside this box otherwise the test will fail. This
    * can help to make sure the robot actually walked to an expected location. See {@link ICubRampsTest}
    * for an example.
    */
   public BoundingBox3D getFinalBoundingBox()
   {
      return null;
   }

   /**
    * Overwrite this if you are using a custom robot model. This can be used to change robot parameters
    * for a specific test. See {@link ICubWobblyFeetTest} for an example of how to use this.
    * </p>
    * This method is called when setting up the test. Do not call this from your test. To access the
    * robot model use {@link #getRobotModel()}.
    */
   public IcubRobotModel createRobotModel()
   {
      return new IcubRobotModel(removeJointLimits());
   }

   /**
    * Overwrite this if you want the robot to start in a location different of the origin.
    */
   public DRCStartingLocation getStartingLocation()
   {
      return DRCObstacleCourseStartingLocation.DEFAULT;
   }

   /**
    * Overwrite this if you would like to add a push controller to the test that will allow you to apply
    * forces to the chest of the robot during the test. To get the controller from your test call
    * {@link #getPushRobotController()}.
    */
   public boolean createPushController()
   {
      return false;
   }

   /**
    * To call this method from your test you must overwrite the method {@link #createPushController()}
    * to return {@code true}. In that case a push controller will be created that can be used to push
    * the robot during a test.
    */
   public PushRobotController getPushRobotController()
   {
      if (!createPushController())
      {
         throw new RuntimeException("Can not get push robot controller.");
      }
      if (pushRobotController == null)
      {
         throw new RuntimeException("Push controller was not created yet.");
      }
      return pushRobotController;
   }

   /**
    * Get the test helper to gain access to SCS and YoVariables.
    */
   public DRCSimulationTestHelper getTestHelper()
   {
      if (drcSimulationTestHelper == null)
      {
         throw new RuntimeException("Test Helper was not created yet.");
      }
      return drcSimulationTestHelper;
   }

   /**
    * Get the robot model to access robot parameters.
    */
   public IcubRobotModel getRobotModel()
   {
      if (robotModel == null)
      {
         throw new RuntimeException("Robot model was not created yet.");
      }
      return robotModel;
   }

   /**
    * Send a packet to the controller for execution.
    */
   public void sendPacket(Packet<?> packet)
   {
      getTestHelper().send(packet);
   }

   /**
    * Simulate for the given time and assert that the simulation was successful.
    */
   public void simulate(double time)
   {
      try
      {
         Assert.assertTrue(getTestHelper().simulateAndBlockAndCatchExceptions(time));
      }
      catch (SimulationExceededMaximumTimeException e)
      {
         e.printStackTrace();
         Assert.fail("Simulation failed.");
      }
   }

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

      DRCStartingLocation startingLocation = getStartingLocation();

      // Suppress errors on creation of the robot model and the test helper.
      PrintStream originalErrorStream = System.err;
      PrintStream supressStream = new PrintStream(new ByteArrayOutputStream());
      System.setErr(supressStream);
      robotModel = createRobotModel();
      drcSimulationTestHelper = new DRCSimulationTestHelper(simulationTestingParameters, robotModel, getEnvironment());
      drcSimulationTestHelper.setStartingLocation(startingLocation);
      System.setErr(originalErrorStream);

      drcSimulationTestHelper.createSimulation(name.getMethodName());

      OffsetAndYawRobotInitialSetup startingLocationOffset = startingLocation.getStartingLocationOffset();
      Point3D cameraFocus = new Point3D(startingLocationOffset.getAdditionalOffset());
      cameraFocus.addZ(0.4);
      RigidBodyTransform transform = new RigidBodyTransform();
      transform.setRotationYawAndZeroTranslation(startingLocationOffset.getYaw());
      transform.transform(getCameraPosion());

      BoundingBox3D box = getFinalBoundingBox();
      if (box != null)
      {
         Graphics3DObject boxGraphics = new Graphics3DObject();
         Point3DReadOnly minPoint = box.getMinPoint();
         Point3DReadOnly maxPoint = box.getMaxPoint();
         Point3D center = new Point3D();
         center.interpolate(minPoint, maxPoint, 0.5);
         Point3D dimensions = new Point3D();
         dimensions.sub(maxPoint, minPoint);
         boxGraphics.translate(center);
         YoAppearanceRGBColor cubeAppearance = new YoAppearanceRGBColor(Color.GREEN, 0.9);
         boxGraphics.addCube(dimensions.getX(), dimensions.getY(), dimensions.getZ(), true, cubeAppearance);
         drcSimulationTestHelper.getSimulationConstructionSet().addStaticLinkGraphics(boxGraphics);
      }

      if (createPushController())
      {
         Vector3D pushLocation = new Vector3D(0.0, 0.0, 0.0);
         FullHumanoidRobotModel fullRobotModel = drcSimulationTestHelper.getControllerFullRobotModel();
         String chestParentJointName = fullRobotModel.getChest().getParentJoint().getName();
         pushRobotController = new PushRobotController(drcSimulationTestHelper.getRobot(), chestParentJointName, pushLocation);
         SimulationConstructionSet scs = drcSimulationTestHelper.getSimulationConstructionSet();
         scs.addYoGraphic(pushRobotController.getForceVisualizer());
      }

      drcSimulationTestHelper.setupCameraForUnitTest(cameraFocus, getCameraPosion());
      ThreadTools.sleep(1000);
   }

   @After
   public void finishTest()
   {
      BoundingBox3D boundingBox = getFinalBoundingBox();
      if (boundingBox != null && !simulationTestingParameters.getKeepSCSUp())
      {
         drcSimulationTestHelper.assertRobotsRootJointIsInBoundingBox(boundingBox);
      }

      if (simulationTestingParameters.getKeepSCSUp())
      {
         if (exportJointDataWhenDebugging)
         {
            exportJointData();
         }
         ThreadTools.sleepForever();
      }

      if (drcSimulationTestHelper != null)
      {
         drcSimulationTestHelper.destroySimulation();
      }

      drcSimulationTestHelper = null;
      pushRobotController = null;
      robotModel = null;

      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
   }
}
