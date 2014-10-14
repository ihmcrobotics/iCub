package it.iit.iCub.parameters;

import us.ihmc.robotSide.RobotSide;
import us.ihmc.robotSide.SideDependentList;

public class IcubOrderedJointMap 
{
	public final static int l_hip_pitch = 0;
	public final static int l_hip_roll = 1;
	public final static int l_hip_yaw = 2;
	public final static int l_knee = 3;
	public final static int l_ankle_pitch = 4;
	public final static int l_ankle_roll= 5;
	public final static int r_hip_pitch = 6;
	public final static int r_hip_roll = 7;
	public final static int r_hip_yaw = 8;
	public final static int r_knee = 9;
	public final static int r_ankle_pitch = 10;
	public final static int r_ankle_roll = 11;
	public final static int torso_pitch = 12;
	public final static int torso_roll = 13;
	public final static int torso_yaw = 14;
	public final static int l_shoulder_pitch = 15;
	public final static int l_shoulder_roll = 16;
	public final static int l_shoulder_yaw = 17;
	public final static int l_elbow = 18;
	public final static int l_wrist_prosup = 19;
	public final static int l_wrist_pitch = 20;
	public final static int l_wrist_yaw = 21;
	public final static int neck_pitch = 22;
	public final static int neck_roll = 23;
	public final static int neck_yaw = 24;
	public final static int r_shoulder_pitch = 25;
	public final static int r_shoulder_roll = 26;
	public final static int r_shoulder_yaw = 27;
	public final static int r_elbow = 28;
	public final static int r_wrist_prosup = 29;
	public final static int r_wrist_pitch = 30;
	public final static int r_wrist_yaw = 31;
	
	public final static int numberOfJoints = r_wrist_yaw + 1;
	
	public static String[]  jointNames = new String[numberOfJoints];
	static 
	{
		jointNames[l_hip_pitch] = "l_hip_pitch";
		jointNames[l_hip_roll] = "l_hip_roll";
		jointNames[l_hip_yaw] = "l_hip_yaw";
		jointNames[l_knee] = "l_knee";
		jointNames[l_ankle_pitch] = "l_ankle_pitch";
		jointNames[l_ankle_roll] = "l_ankle_roll";
		jointNames[r_hip_pitch] = "r_hip_pitch";
		jointNames[r_hip_roll] = "r_hip_roll";
		jointNames[r_hip_yaw] = "r_hip_yaw";
		jointNames[r_knee] = "r_knee";
		jointNames[r_ankle_pitch] = "r_ankle_pitch";
		jointNames[r_ankle_roll] = "r_ankle_roll";
		jointNames[torso_pitch] = "torso_pitch";
		jointNames[torso_roll] = "torso_roll";
		jointNames[torso_yaw] = "torso_yaw";
		jointNames[l_shoulder_pitch] = "l_shoulder_pitch";
		jointNames[l_shoulder_roll] = "l_shoulder_roll";
		jointNames[l_shoulder_yaw] = "l_shoulder_yaw";
		jointNames[l_elbow] = "l_elbow";
		jointNames[l_wrist_prosup] = "l_wrist_prosup";
		jointNames[l_wrist_pitch] = "l_wrist_pitch";
		jointNames[l_wrist_yaw] = "l_wrist_yaw";
		jointNames[neck_pitch] = "neck_pitch";
		jointNames[neck_roll] = "neck_roll";
		jointNames[neck_yaw] = "neck_yaw";
		jointNames[r_shoulder_pitch] = "r_shoulder_pitch";
		jointNames[r_shoulder_roll] = "r_shoulder_roll";
		jointNames[r_shoulder_yaw] = "r_shoulder_yaw";
		jointNames[r_elbow] = "r_elbow";
		jointNames[r_wrist_prosup] = "r_wrist_prosup";
		jointNames[r_wrist_pitch] = "r_wrist_pitch";
		jointNames[r_wrist_yaw] = "r_wrist_yaw";
	}
	
	public static final SideDependentList<String[]> forcedSideDependentJointNames = new SideDependentList<String[]>();
	static 
	{
		String[] jointNamesRight = new String[numberOfJoints];
		jointNamesRight[l_hip_pitch] = jointNames[r_hip_pitch];
		jointNamesRight[l_hip_roll] = jointNames[r_hip_roll];
		jointNamesRight[l_hip_yaw] = jointNames[r_hip_yaw];
		jointNamesRight[l_knee] = jointNames[r_knee];
		jointNamesRight[l_ankle_pitch] = jointNames[r_ankle_pitch];
		jointNamesRight[l_ankle_roll] = jointNames[r_ankle_roll];
		jointNamesRight[r_hip_pitch] = jointNames[r_hip_pitch];
		jointNamesRight[r_hip_roll] = jointNames[r_hip_roll];
		jointNamesRight[r_hip_yaw] = jointNames[r_hip_yaw];
		jointNamesRight[r_knee] = jointNames[r_knee];
		jointNamesRight[r_ankle_pitch] = jointNames[r_ankle_pitch];
		jointNamesRight[r_ankle_roll] = jointNames[r_ankle_roll];
		jointNamesRight[torso_pitch] = jointNames[torso_pitch];
		jointNamesRight[torso_roll] = jointNames[torso_roll];
		jointNamesRight[torso_yaw] = jointNames[torso_yaw];
		jointNamesRight[l_shoulder_pitch] = jointNames[r_shoulder_pitch];
		jointNamesRight[l_shoulder_roll] = jointNames[r_shoulder_roll];
		jointNamesRight[l_shoulder_yaw] = jointNames[r_shoulder_yaw];
		jointNamesRight[l_elbow] = jointNames[r_elbow];
		jointNamesRight[l_wrist_prosup] = jointNames[r_wrist_prosup];
		jointNamesRight[l_wrist_pitch] = jointNames[r_wrist_pitch];
		jointNamesRight[l_wrist_yaw] = jointNames[r_wrist_yaw];
		jointNamesRight[neck_pitch] = jointNames[neck_pitch];
		jointNamesRight[neck_roll] = jointNames[neck_roll];
		jointNamesRight[neck_yaw] = jointNames[neck_yaw];
		jointNamesRight[r_shoulder_pitch] = jointNames[r_shoulder_pitch];
		jointNamesRight[r_shoulder_roll] = jointNames[r_shoulder_roll];
		jointNamesRight[r_shoulder_yaw] = jointNames[r_shoulder_yaw];
		jointNamesRight[r_elbow] = jointNames[r_elbow];
		jointNamesRight[r_wrist_prosup] = jointNames[r_wrist_prosup];
		jointNamesRight[r_wrist_pitch] = jointNames[r_wrist_pitch];
		jointNamesRight[r_wrist_yaw] = jointNames[r_wrist_yaw];

		forcedSideDependentJointNames.put(RobotSide.RIGHT, jointNamesRight);

		String[] jointNamesLeft = new String[numberOfJoints];
		jointNamesLeft[l_hip_pitch] = jointNames[l_hip_pitch];
		jointNamesLeft[l_hip_roll] = jointNames[l_hip_roll]; 
		jointNamesLeft[l_hip_yaw] = jointNames[l_hip_yaw];
		jointNamesLeft[l_knee] = jointNames[l_knee];
		jointNamesLeft[l_ankle_pitch] = jointNames[l_ankle_pitch];
		jointNamesLeft[l_ankle_roll] = jointNames[l_ankle_roll];
		jointNamesLeft[r_hip_pitch] = jointNames[l_hip_pitch];
		jointNamesLeft[r_hip_roll] = jointNames[l_hip_roll];
		jointNamesLeft[r_hip_yaw] = jointNames[l_hip_yaw];
		jointNamesLeft[r_knee] = jointNames[l_knee];
		jointNamesLeft[r_ankle_pitch] = jointNames[l_ankle_pitch];
		jointNamesLeft[r_ankle_roll] = jointNames[l_ankle_roll];
		jointNamesLeft[torso_pitch] = jointNames[torso_pitch];
		jointNamesLeft[torso_roll] = jointNames[torso_roll];
		jointNamesLeft[torso_yaw] = jointNames[torso_yaw];
		jointNamesLeft[l_shoulder_pitch] = jointNames[l_shoulder_pitch];
		jointNamesLeft[l_shoulder_roll] = jointNames[l_shoulder_roll];
		jointNamesLeft[l_shoulder_yaw] = jointNames[l_shoulder_yaw];
		jointNamesLeft[l_elbow] = jointNames[l_elbow];
		jointNamesLeft[l_wrist_prosup] = jointNames[l_wrist_prosup];
		jointNamesLeft[l_wrist_pitch] = jointNames[l_wrist_pitch];
		jointNamesLeft[l_wrist_yaw] = jointNames[l_wrist_yaw];
		jointNamesLeft[neck_pitch] = jointNames[neck_pitch];
		jointNamesLeft[neck_roll] = jointNames[neck_roll];
		jointNamesLeft[neck_yaw] = jointNames[neck_yaw];
		jointNamesLeft[r_shoulder_pitch] = jointNames[l_shoulder_pitch];
		jointNamesLeft[r_shoulder_roll] = jointNames[l_shoulder_roll];
		jointNamesLeft[r_shoulder_yaw] = jointNames[l_shoulder_yaw];
		jointNamesLeft[r_elbow] = jointNames[l_elbow];
		jointNamesLeft[r_wrist_prosup] = jointNames[l_wrist_prosup];
		jointNamesLeft[r_wrist_pitch] = jointNames[l_wrist_pitch];
		jointNamesLeft[r_wrist_yaw] = jointNames[l_wrist_yaw];

		forcedSideDependentJointNames.put(RobotSide.LEFT, jointNamesLeft);
	}
}
