
package org.usfirst.frc.team5806.robot;

import java.io.File;
import java.io.IOException;
import java.nio.charset.Charset;
import java.nio.file.Files;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Date;

import java.util.Map;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot {
	Command autonomousCommand;
	SendableChooser<Command> chooser = new SendableChooser<>();

	RobotDrive robotdrive;
	Joystick joy;
	Encoder leftencoder;
	Encoder rightencoder;
	static final double PERIOD_OF_OSCILLATION = 0.05;
	static final double TICKS_PER_INCH = 7.676;
	FileReader reader;
	
	
	AHRS ahrs;
	
	double integral = 0.0;
	
	public void robotInit() {
		reader = new FileReader("/home/lvuser/TestFile");

//		robotdrive = new RobotDrive(1, 3);
		robotdrive = new RobotDrive(3, 1);
		joy = new Joystick(0);
		
		leftencoder = new Encoder(2,3);
		rightencoder = new Encoder(0,1);
		rightencoder.setReverseDirection(true);
		
		ahrs = new AHRS(SPI.Port.kMXP);
		
		System.out.println("INIT");
	}


	/**
	 * This function is called once each time the robot enters Disabled mode.
	 * You can use it to reset any subsystem information you want to clear when
	 * the robot is disabled.
	 */
	@Override
	public void disabledInit() {

	}

	@Override
	public void disabledPeriodic() {
	}
	
	@Override
	public void autonomousInit() {
		System.out.println("AUTO_INIT");
		rightencoder.reset();
		leftencoder.reset();
		
		String gameData;
		gameData = DriverStation.getInstance().getGameSpecificMessage();
		/*if(gameData.charAt(0) == 'L')
		{
			//Put left auto code here
		} else {
			//Put right auto code here
		}*/
		
		try {
			//goForward(1000, 0.8);
			System.out.println("CALL_SPLINE");
			spline(gameData.charAt(0));
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}//*/
		robotdrive.tankDrive(0, 0);
	}
	
	double[] pathLeft = {0.010283, 0.047991, 4.666804, 0.000494, 0.020134, 0.121684, 7.481180, 0.001692, 0.030274, 0.201689, 7.890134, 0.003737, 0.040048, 0.281439, 8.158959, 0.006488, 0.050027, 0.360616, 7.934228, 0.010087, 0.060109, 0.441112, 7.984403, 0.014534, 0.070124, 0.521862, 8.063158, 0.019760, 0.080096, 0.602302, 8.066086, 0.025767, 0.090041, 0.682608, 8.075306, 0.032555, 0.100054, 0.763242, 8.053012, 0.040197, 0.110036, 0.844214, 8.111478, 0.048625, 0.120069, 0.925472, 8.099155, 0.057910, 0.130005, 1.006763, 8.181448, 0.067913, 0.140054, 1.088358, 8.120102, 0.078849, 0.150016, 1.170317, 8.227058, 0.090508, 0.160018, 1.252360, 8.202753, 0.103034, 0.170000, 1.334777, 8.256216, 0.116359, 0.180015, 1.417557, 8.265639, 0.130555, 0.190011, 1.500716, 8.319582, 0.145556, 0.200034, 1.584249, 8.334334, 0.161434, 0.210037, 1.668173, 8.389310, 0.178122, 0.220024, 1.752320, 8.425665, 0.195623, 0.230034, 1.836879, 8.447512, 0.214010, 0.240027, 1.921866, 8.504680, 0.233215, 0.250004, 2.007130, 8.545741, 0.253241, 0.260001, 2.092840, 8.573818, 0.274162, 0.270015, 2.179151, 8.619558, 0.295983, 0.280010, 2.265917, 8.680193, 0.318633, 0.290020, 2.353143, 8.714017, 0.342187, 0.300013, 2.440841, 8.776590, 0.366577, 0.310016, 2.529014, 8.814016, 0.391876, 0.320002, 2.617674, 8.878644, 0.418016, 0.330024, 2.706944, 8.907819, 0.445143, 0.340025, 2.796829, 8.987209, 0.473116, 0.350007, 2.887096, 9.042937, 0.501935, 0.360019, 2.977982, 9.078121, 0.531749, 0.370009, 3.069494, 9.160159, 0.562414, 0.380001, 3.161521, 9.209854, 0.594005, 0.390016, 3.254281, 9.262191, 0.626596, 0.400007, 3.347672, 9.347436, 0.660043, 0.410017, 3.441690, 9.392655, 0.694493, 0.420001, 3.536344, 9.480322, 0.729801, 0.430000, 3.631633, 9.529571, 0.766115, 0.440012, 3.727754, 9.601077, 0.803435, 0.450014, 3.824608, 9.683587, 0.841688, 0.460005, 3.922102, 9.757977, 0.880875, 0.470003, 4.020332, 9.825167, 0.921069, 0.480005, 4.119387, 9.903730, 0.962271, 0.490008, 4.219266, 9.984139, 1.004479, 0.500012, 4.319966, 10.066381, 1.047694, 0.510013, 4.421483, 10.150429, 1.091915, 0.520010, 4.523816, 10.236239, 1.137140, 0.530001, 4.626961, 10.323738, 1.183368, 0.540000, 4.730998, 10.404739, 1.230673, 0.550004, 4.836000, 10.495887, 1.279053, 0.560011, 4.941956, 10.588457, 1.328506, 0.570004, 5.048776, 10.690045, 1.378956, 0.580009, 5.156523, 10.768846, 1.430549, 0.590010, 5.265261, 10.872293, 1.483209, 0.600006, 5.374890, 10.968015, 1.536933, 0.610007, 5.485463, 11.056160, 1.591793, 0.620011, 5.597027, 11.151609, 1.647787, 0.630003, 5.709470, 11.253203, 1.704837, 0.640007, 5.822824, 11.330812, 1.763089, 0.650008, 5.937116, 11.428152, 1.822466, 0.660004, 6.052214, 11.514488, 1.882963, 0.670006, 6.168124, 11.589328, 1.944653, 0.680010, 6.284840, 11.665875, 2.007532, 0.690005, 6.402204, 11.742328, 2.071522, 0.700001, 6.520116, 11.796604, 2.136694, 0.710007, 6.638600, 11.841523, 2.203118, 0.720010, 6.757525, 11.888510, 2.270716, 0.730010, 6.876681, 11.915444, 2.339484, 0.740000, 6.955186, 7.858035, 2.408969, 0.750001, 6.914641, -4.054160, 2.478120, 0.760007, 6.870279, -4.433648, 2.546862, 0.770000, 6.830028, -4.027806, 2.615118, 0.780009, 6.793652, -3.634586, 2.683110, 0.790004, 6.760968, -3.269953, 2.750688, 0.800003, 6.731831, -2.914063, 2.817999, 0.810000, 6.706081, -2.575651, 2.885041, 0.820002, 6.683599, -2.247824, 2.951888, 0.830003, 6.664282, -1.931542, 3.018538, 0.840010, 6.648042, -1.622855, 3.085064, 0.850006, 6.634815, -1.323131, 3.151389, 0.860010, 6.624540, -1.027129, 3.217662, 0.870006, 6.617167, -0.737568, 3.283805, 0.880011, 6.612664, -0.450148, 3.349966, 0.890010, 6.611006, -0.165744, 3.416068, 0.900009, 6.612185, 0.117912, 3.482185, 0.910004, 6.616199, 0.401572, 3.548314, 0.920002, 6.623059, 0.686121, 3.614527, 0.930008, 6.632793, 0.972844, 3.680897, 0.940007, 6.645435, 1.264286, 3.747348, 0.950007, 6.661024, 1.559051, 3.813952, 0.960000, 6.679617, 1.860353, 3.880708, 0.970006, 6.701294, 2.166538, 3.947759, 0.980008, 6.726146, 2.484772, 4.015032, 0.990000, 6.754243, 2.811779, 4.082525, 1.000000, 6.785712, 3.146859, 4.150382, 1.010002, 6.820701, 3.498266, 4.218602, 1.020000, 6.859343, 3.864850, 4.287182, 1.030010, 6.901826, 4.244180, 4.356268, 1.040005, 6.948318, 4.651929, 4.425711, 1.050008, 6.999032, 5.069377, 4.495729, 1.060005, 7.054231, 5.521984, 4.566245, 1.070007, 7.114185, 5.993864, 4.637404, 1.080009, 7.178576, 6.437944, 4.709204, 1.090010, 7.243162, 6.458049, 4.781641, 1.100002, 7.306061, 6.295109, 4.854642, 1.110006, 7.367369, 6.128128, 4.928347, 1.120005, 7.427162, 5.979909, 5.002611, 1.130000, 7.485459, 5.832520, 5.077430, 1.140003, 7.542370, 5.689619, 5.152874, 1.150004, 7.597975, 5.559463, 5.228866, 1.160006, 7.652323, 5.433795, 5.305405, 1.170000, 7.705469, 5.317779, 5.382413, 1.180006, 7.757510, 5.201303, 5.460030, 1.190005, 7.808516, 5.101119, 5.538107, 1.200007, 7.858527, 4.999987, 5.616710, 1.210004, 7.907604, 4.909199, 5.695762, 1.220005, 7.955799, 4.819157, 5.775326, 1.230000, 8.003160, 4.738088, 5.855323, 1.240009, 8.049744, 4.654685, 5.935886, 1.250002, 8.095565, 4.584854, 6.016791, 1.260008, 8.140644, 4.505503, 6.098241, 1.270006, 8.185017, 4.437930, 6.180080, 1.280006, 8.228673, 4.365627, 6.262367, 1.290006, 8.271630, 4.295609, 6.345084, 1.300006, 8.313878, 4.225144, 6.428216, 1.310003, 8.355400, 4.153458, 6.511745, 1.320004, 8.396187, 4.078045, 6.595720, 1.330000, 8.436204, 4.003207, 6.680050, 1.340006, 8.475423, 3.919797, 6.764852, 1.350003, 8.513794, 3.838362, 6.849960, 1.360004, 8.551254, 3.745396, 6.935487, 1.370001, 8.587751, 3.650936, 7.021334, 1.380006, 8.623220, 3.545201, 7.107608, 1.390000, 8.657575, 3.437334, 7.194139, 1.400002, 8.687862, 3.028342, 7.281029, 1.410003, 8.613027, -7.482529, 7.367170, 1.420000, 8.486659, -12.640285, 7.452014, 1.430007, 8.361422, -12.515757, 7.535681, 1.440004, 8.237266, -12.419164, 7.618029, 1.450005, 8.114147, -12.310200, 7.699182, 1.460000, 7.991978, -12.222570, 7.779065, 1.470004, 7.870675, -12.125775, 7.857801, 1.480006, 7.750166, -12.048654, 7.935318, 1.490004, 7.630479, -11.970697, 8.011610, 1.500006, 7.511551, -11.891013, 8.086736, 1.510001, 7.393375, -11.823731, 8.160632, 1.520004, 7.275899, -11.743136, 8.233418, 1.530008, 7.159079, -11.678282, 8.305032, 1.540001, 7.043025, -11.613610, 8.375413, 1.550009, 6.927651, -11.528174, 8.444745, 1.560005, 6.812980, -11.471274, 8.512849, 1.570007, 6.699089, -11.387179, 8.579852, 1.580005, 6.585960, -11.315165, 8.645698, 1.590008, 6.473631, -11.229494, 8.710454, 1.600007, 6.362142, -11.149575, 8.774071, 1.610003, 6.251591, -11.059796, 8.836561, 1.620004, 6.141978, -10.959496, 8.897991, 1.630004, 6.033355, -10.863250, 8.958319, 1.640001, 5.925828, -10.755618, 9.017561, 1.650006, 5.819406, -10.636170, 9.075788, 1.660003, 5.714198, -10.524652, 9.132910, 1.670001, 5.610311, -10.391032, 9.189000, 1.680001, 5.507755, -10.255065, 9.244080, 1.690006, 5.406588, -10.111787, 9.298172, 1.700007, 5.306913, -9.966312, 9.351248, 1.710006, 5.208830, -9.809142, 9.403332, 1.720005, 5.112384, -9.645570, 9.454450, 1.730006, 5.017768, -9.461060, 9.504631, 1.740005, 4.927510, -9.026749, 9.553901, 1.750005, 4.843497, -8.400911, 9.602338, 1.760000, 4.765546, -7.798977, 9.649970, 1.770004, 4.693367, -7.215169, 9.696921, 1.780011, 4.626720, -6.660361, 9.743219, 1.790003, 4.565491, -6.127447, 9.788840, 1.800008, 4.509481, -5.598351, 9.833956, 1.810009, 4.458521, -5.095456, 9.878545, 1.820000, 4.412542, -4.601856, 9.922633, 1.830010, 4.371387, -4.111564, 9.966389, 1.840011, 4.334968, -3.641528, 10.009743, 1.850009, 4.303249, -3.172568, 10.052767, 1.860010, 4.276140, -2.710430, 10.095534, 1.870011, 4.253592, -2.254862, 10.138071, 1.880005, 4.235568, -1.803412, 10.180401, 1.890010, 4.222028, -1.353299, 10.222646, 1.900001, 4.212955, -0.908152, 10.264734, 1.910005, 4.208335, -0.461759, 10.306835, 1.920007, 4.208165, -0.017006, 10.348925, 1.930002, 4.212453, 0.428974, 10.391029, 1.940008, 4.221215, 0.875654, 10.433269, 1.950010, 4.234482, 1.326372, 10.475622, 1.960003, 4.252271, 1.780223, 10.518114, 1.970004, 4.274637, 2.236382, 10.560865, 1.980009, 4.301655, 2.700497, 10.603902, 1.990001, 4.333368, 3.173720, 10.647203, 2.000010, 4.369879, 3.648098, 10.690938, 2.010007, 4.411292, 4.142577, 10.735037, 2.020009, 4.457703, 4.640069, 10.779624, 2.030011, 4.509278, 5.156496, 10.824726, 2.040007, 4.566147, 5.689103, 10.870369, 2.050002, 4.628491, 6.237459, 10.916632, 2.060001, 4.696557, 6.807407, 10.963591, 2.070000, 4.742961, 4.640532, 11.011020, 2.080002, 4.707977, -3.497675, 11.058108, 2.090007, 4.667022, -4.093527, 11.104801, 2.100003, 4.625765, -4.127663, 11.151037, 2.110010, 4.584062, -4.167185, 11.196912, 2.120005, 4.541813, -4.226954, 11.242307, 2.130009, 4.498934, -4.286039, 11.287316, 2.140009, 4.455331, -4.360304, 11.331870, 2.150003, 4.410974, -4.438424, 11.375953, 2.160001, 4.365795, -4.518785, 11.419602, 2.170002, 4.319734, -4.605907, 11.462801, 2.180002, 4.272770, -4.696168, 11.505531, 2.190001, 4.224889, -4.788710, 11.547775, 2.200009, 4.176054, -4.879448, 11.589570, 2.210011, 4.126266, -4.977743, 11.630841, 2.220004, 4.075569, -5.073169, 11.671569, 2.230000, 4.023944, -5.164646, 11.711792, 2.240012, 3.971330, -5.255255, 11.751551, 2.250007, 3.917786, -5.356955, 11.790711, 2.260014, 3.863334, -5.441477, 11.829371, 2.270015, 3.807958, -5.537047, 11.867454, 2.280008, 3.751730, -5.627077, 11.904942, 2.290005, 3.694637, -5.710725, 11.941880, 2.300006, 3.636660, -5.797133, 11.978250, 2.310008, 3.577827, -5.881838, 12.014037, 2.320010, 3.518169, -5.964772, 12.049225, 2.330009, 3.457716, -6.045912, 12.083798, 2.340003, 3.396501, -6.125217, 12.117743, 2.350009, 3.334499, -6.196735, 12.151107, 2.360006, 3.271738, -6.278166, 12.183813, 2.370011, 3.208246, -6.345714, 12.215913, 2.380003, 3.144057, -6.424059, 12.247329, 2.390000, 3.079198, -6.487662, 12.278112, 2.400001, 3.013632, -6.555920, 12.308252, 2.410005, 2.947382, -6.622544, 12.337736, 2.420010, 2.880475, -6.687566, 12.366555, 2.430014, 2.812935, -6.751026, 12.394696, 2.440017, 2.744788, -6.812929, 12.422151, 2.450016, 2.676061, -6.873324, 12.448910, 2.460010, 2.606780, -6.932220, 12.474962, 2.470024, 2.536880, -6.980338, 12.500366, 2.480003, 2.466478, -7.054913, 12.524979, 2.490028, 2.395495, -7.080157, 12.548995, 2.500015, 2.323959, -7.162942, 12.572205, 2.510020, 2.251989, -7.193927, 12.594734, 2.520011, 2.179507, -7.254699, 12.616510, 2.530019, 2.106530, -7.291865, 12.637592, 2.540011, 2.033080, -7.350835, 12.657906, 2.550020, 1.959173, -7.384152, 12.677516, 2.560009, 1.884834, -7.441430, 12.696345, 2.570016, 1.810077, -7.470719, 12.714458, 2.580000, 1.734931, -7.526442, 12.731780, 2.590001, 1.659412, -7.551440, 12.748375, 2.600020, 1.583378, -7.588811, 12.764239, 2.610013, 1.507009, -7.642502, 12.779298, 2.620024, 1.430318, -7.660385, 12.793618, 2.630004, 1.353343, -7.713108, 12.807124, 2.640002, 1.276099, -7.725674, 12.819883, 2.650023, 1.198396, -7.754555, 12.831891, 2.660005, 1.120470, -7.806522, 12.843076, 2.670009, 1.042338, -7.809775, 12.853504, 2.680041, 0.963759, -7.832756, 12.863172, 2.690026, 0.885023, -7.885693, 12.872009, 2.700040, 0.806145, -7.877092, 12.880081, 2.710092, 0.726800, -7.892923, 12.887387, 2.720084, 0.647364, -7.950537, 12.893855, 2.730119, 0.567839, -7.924823, 12.899554, 2.740064, 0.488355, -7.992129, 12.904410, 2.750052, 0.408939, -7.951276, 12.908495, 2.760118, 0.328940, -7.947542, 12.911806, 2.770014, 0.249223, -8.055009, 12.914272, 2.780524, 0.167670, -7.759866, 12.916035, 2.790295, 0.086576, -8.299381, 12.916880};
	
	double[] pathRight = {0.010283, 0.047988, 4.666529, 0.000493, 0.020134, 0.121655, 7.478561, 0.001692, 0.030274, 0.201573, 7.881556, 0.003736, 0.040048, 0.281138, 8.139914, 0.006484, 0.050027, 0.359991, 7.901898, 0.010076, 0.060109, 0.439981, 7.934125, 0.014512, 0.070124, 0.520002, 7.990449, 0.019720, 0.080096, 0.599460, 7.967585, 0.025698, 0.090041, 0.678493, 7.947286, 0.032445, 0.100054, 0.757518, 7.892390, 0.040030, 0.110036, 0.836509, 7.912928, 0.048380, 0.120069, 0.915374, 7.860627, 0.057564, 0.130005, 0.993834, 7.896532, 0.067439, 0.140054, 1.072115, 7.790345, 0.078212, 0.150016, 1.150242, 7.842310, 0.089671, 0.160018, 1.227911, 7.765501, 0.101952, 0.170000, 1.305368, 7.759323, 0.114983, 0.180015, 1.382567, 7.708420, 0.128830, 0.190011, 1.459492, 7.695899, 0.143418, 0.200034, 1.536105, 7.643949, 0.158814, 0.210037, 1.612393, 7.625902, 0.174944, 0.220024, 1.688172, 7.587854, 0.191804, 0.230034, 1.763588, 7.534044, 0.209457, 0.240027, 1.838624, 7.508942, 0.227830, 0.250004, 1.913122, 7.466735, 0.246918, 0.260001, 1.987205, 7.410651, 0.266784, 0.270015, 2.060975, 7.367233, 0.287421, 0.280010, 2.134283, 7.333708, 0.308755, 0.290020, 2.207103, 7.274869, 0.330848, 0.300013, 2.279422, 7.237501, 0.353624, 0.310016, 2.351217, 7.176794, 0.377145, 0.320002, 2.422472, 7.135737, 0.401336, 0.330024, 2.493261, 7.063654, 0.426322, 0.340025, 2.563559, 7.028811, 0.451961, 0.350007, 2.633160, 6.972620, 0.478245, 0.360019, 2.702222, 6.898270, 0.505299, 0.370009, 2.770724, 6.856920, 0.532979, 0.380001, 2.838557, 6.788532, 0.561343, 0.390016, 2.905854, 6.719614, 0.590445, 0.400007, 2.972511, 6.671695, 0.620143, 0.410017, 3.038499, 6.592351, 0.650558, 0.420001, 3.103797, 6.540083, 0.681547, 0.430000, 3.168376, 6.458403, 0.713228, 0.440012, 3.232340, 6.389122, 0.745589, 0.450014, 3.295593, 6.324011, 0.778551, 0.460005, 3.358043, 6.250525, 0.812102, 0.470003, 3.419725, 6.169591, 0.846291, 0.480005, 3.480665, 6.092895, 0.881104, 0.490008, 3.540830, 6.014260, 0.916526, 0.500012, 3.600188, 5.933708, 0.952541, 0.510013, 3.658709, 5.851271, 0.989133, 0.520010, 3.716362, 5.766999, 1.026286, 0.530001, 3.773121, 5.680968, 1.063983, 0.540000, 3.829004, 5.588851, 1.102269, 0.550004, 3.884025, 5.499818, 1.141125, 0.560011, 3.938154, 5.409244, 1.180534, 0.570004, 3.991326, 5.321288, 1.220417, 0.580009, 4.043560, 5.220529, 1.260874, 0.590010, 4.094875, 5.130726, 1.301829, 0.600006, 4.145217, 5.036557, 1.343262, 0.610007, 4.194612, 4.939023, 1.385212, 0.620011, 4.243088, 4.845543, 1.427661, 0.630003, 4.290612, 4.756185, 1.470533, 0.640007, 4.337224, 4.659224, 1.513923, 0.650008, 4.382968, 4.574019, 1.557757, 0.660004, 4.427840, 4.489056, 1.602017, 0.670006, 4.471905, 4.405794, 1.646743, 0.680010, 4.515234, 4.330848, 1.691917, 0.690005, 4.557864, 4.265100, 1.737473, 0.700001, 4.599869, 4.202415, 1.783451, 0.710007, 4.641389, 4.149587, 1.829891, 0.720010, 4.682530, 4.112722, 1.876732, 0.730010, 4.723395, 4.086426, 1.923967, 0.740000, 4.736524, 1.314181, 1.971287, 0.750001, 4.670328, -6.619014, 2.017993, 0.760007, 4.604398, -6.589239, 2.064064, 0.770000, 4.544124, -6.031378, 2.109475, 0.780009, 4.489268, -5.481072, 2.154405, 0.790004, 4.439656, -4.963619, 2.198780, 0.800003, 4.395160, -4.450102, 2.242727, 0.810000, 4.355621, -3.954930, 2.286271, 0.820002, 4.320930, -3.468569, 2.329488, 0.830003, 4.290992, -2.993517, 2.372402, 0.840010, 4.265729, -2.524577, 2.415088, 0.850006, 4.245087, -2.064846, 2.457525, 0.860010, 4.229011, -1.606983, 2.499832, 0.870006, 4.217454, -1.156189, 2.541989, 0.880011, 4.210385, -0.706533, 2.584114, 0.890010, 4.207782, -0.260303, 2.626187, 0.900009, 4.209635, 0.185265, 2.668280, 0.910004, 4.215936, 0.630479, 2.710419, 0.920002, 4.226693, 1.075918, 2.752675, 0.930008, 4.241930, 1.522746, 2.795121, 0.940007, 4.261670, 1.974081, 2.837735, 0.950007, 4.285937, 2.426948, 2.880591, 0.960000, 4.314774, 2.885480, 2.923712, 0.970006, 4.348256, 3.346301, 2.967220, 0.980008, 4.386458, 3.819541, 3.011092, 0.990000, 4.429419, 4.299302, 3.055353, 1.000000, 4.477256, 4.783661, 3.100126, 1.010002, 4.530108, 5.284222, 3.145435, 1.020000, 4.588079, 5.798208, 3.191308, 1.030010, 4.651352, 6.321054, 3.237867, 1.040005, 4.720062, 6.874961, 3.285040, 1.050008, 4.794401, 7.431083, 3.333003, 1.060005, 4.874628, 8.025632, 3.381731, 1.070007, 4.960994, 8.634446, 3.431353, 1.080009, 5.053373, 9.236138, 3.481897, 1.090010, 5.148795, 9.541461, 3.533389, 1.100002, 5.245838, 9.712305, 3.585804, 1.110006, 5.344501, 9.861992, 3.639272, 1.120005, 5.444737, 10.024652, 3.693714, 1.130000, 5.546395, 10.170678, 3.749151, 1.140003, 5.649470, 10.304744, 3.805661, 1.150004, 5.753903, 10.441574, 3.863210, 1.160006, 5.859587, 10.566310, 3.921817, 1.170000, 5.966412, 10.688968, 3.981445, 1.180006, 6.074369, 10.789890, 4.042221, 1.190005, 6.183402, 10.904363, 4.104049, 1.200007, 6.293404, 10.997715, 4.166998, 1.210004, 6.404325, 11.095480, 4.231021, 1.220005, 6.516115, 11.178139, 4.296187, 1.230000, 6.628730, 11.266351, 4.362446, 1.240009, 6.742179, 11.335731, 4.429922, 1.250002, 6.856378, 11.426968, 4.498444, 1.260008, 6.971296, 11.485680, 4.568194, 1.270006, 7.086958, 11.567715, 4.639054, 1.280006, 7.203294, 11.633731, 4.711087, 1.290006, 7.320341, 11.704552, 4.784291, 1.300006, 7.438091, 11.775928, 4.858666, 1.310003, 7.556542, 11.848632, 4.934208, 1.320004, 7.675746, 11.918555, 5.010978, 1.330000, 7.795714, 12.001314, 5.088906, 1.340006, 7.916511, 12.073029, 5.168115, 1.350003, 8.038160, 12.169053, 5.248469, 1.360004, 8.160687, 12.250694, 5.330089, 1.370001, 8.284178, 12.353344, 5.412902, 1.380006, 8.408720, 12.448274, 5.497030, 1.390000, 8.534363, 12.570734, 5.582330, 1.400002, 8.658307, 12.392895, 5.668923, 1.410003, 8.678276, 1.996681, 5.755717, 1.420000, 8.644655, -3.363008, 5.842140, 1.430007, 8.609863, -3.477085, 5.928293, 1.440004, 8.573989, -3.588359, 6.014008, 1.450005, 8.537119, -3.686515, 6.099391, 1.460000, 8.499313, -3.782385, 6.184346, 1.470004, 8.460620, -3.867843, 6.268983, 1.480006, 8.421083, -3.952981, 6.353210, 1.490004, 8.380765, -4.032403, 6.437004, 1.500006, 8.339692, -4.106729, 6.520413, 1.510001, 8.297895, -4.181901, 6.603349, 1.520004, 8.255379, -4.250000, 6.685934, 1.530008, 8.212140, -4.322468, 6.768081, 1.540001, 8.168222, -4.394855, 6.849706, 1.550009, 8.123585, -4.460161, 6.931007, 1.560005, 8.078218, -4.538379, 7.011760, 1.570007, 8.032121, -4.608859, 7.092094, 1.580005, 7.985250, -4.688130, 7.171931, 1.590008, 7.937567, -4.766745, 7.251331, 1.600007, 7.889033, -4.853750, 7.330216, 1.610003, 7.839620, -4.943391, 7.408579, 1.620004, 7.789251, -5.036043, 7.486485, 1.630004, 7.737865, -5.139111, 7.563856, 1.640001, 7.685418, -5.246168, 7.640689, 1.650006, 7.631814, -5.357350, 7.717051, 1.660003, 7.577003, -5.483030, 7.792794, 1.670001, 7.520934, -5.608143, 7.867986, 1.680001, 7.463502, -5.742944, 7.942625, 1.690006, 7.404624, -5.884907, 8.016707, 1.700007, 7.344249, -6.036810, 8.090158, 1.710006, 7.282329, -6.192598, 8.162975, 1.720005, 7.218788, -6.354656, 8.235156, 1.730006, 7.153776, -6.500875, 8.306698, 1.740005, 7.090819, -6.296278, 8.377599, 1.750005, 7.032693, -5.812364, 8.447929, 1.760000, 6.979238, -5.348243, 8.517686, 1.770004, 6.930151, -4.906778, 8.587014, 1.780011, 6.885188, -4.493381, 8.655911, 1.790003, 6.844193, -4.102591, 8.724302, 1.800008, 6.806956, -3.721889, 8.792404, 1.810009, 6.773303, -3.364994, 8.860144, 1.820000, 6.743124, -3.020473, 8.927518, 1.830010, 6.716263, -2.683458, 8.994745, 1.840011, 6.692615, -2.364580, 9.061677, 1.850009, 6.672113, -2.050686, 9.128385, 1.860010, 6.654659, -1.745065, 9.194941, 1.870011, 6.640192, -1.446740, 9.261344, 1.880005, 6.628660, -1.153873, 9.327591, 1.890010, 6.620015, -0.864017, 9.393828, 1.900001, 6.614232, -0.578819, 9.459907, 1.910005, 6.611290, -0.294070, 9.526046, 1.920007, 6.611182, -0.010881, 9.592171, 1.930002, 6.613912, 0.273158, 9.658279, 1.940008, 6.619496, 0.558038, 9.724518, 1.950010, 6.627964, 0.846650, 9.790810, 1.960003, 6.639344, 1.138788, 9.857155, 1.970004, 6.653691, 1.434610, 9.923700, 1.980009, 6.671082, 1.738189, 9.990443, 1.990001, 6.691575, 2.050950, 10.057307, 2.000010, 6.715278, 2.368255, 10.124516, 2.010007, 6.742300, 2.703087, 10.191917, 2.020009, 6.772756, 3.044922, 10.259660, 2.030011, 6.806814, 3.405126, 10.327742, 2.040007, 6.844621, 3.782165, 10.396161, 2.050002, 6.886369, 4.176845, 10.464991, 2.060001, 6.932300, 4.593717, 10.534305, 2.070000, 6.942388, 1.008788, 10.603727, 2.080002, 6.831659, -11.070826, 10.672057, 2.090007, 6.712566, -11.903491, 10.739215, 2.100003, 6.593828, -11.879306, 10.805123, 2.110010, 6.475513, -11.822641, 10.869926, 2.120005, 6.357747, -11.782546, 10.933472, 2.130009, 6.240636, -11.706005, 10.995905, 2.140009, 6.124208, -11.642626, 11.057148, 2.150003, 6.008617, -11.566255, 11.117198, 2.160001, 5.893864, -11.477553, 11.176124, 2.170002, 5.779941, -11.391931, 11.233926, 2.180002, 5.666901, -11.303317, 11.290598, 2.190001, 5.554789, -11.212567, 11.346139, 2.200009, 5.443570, -11.112655, 11.400620, 2.210011, 5.333276, -11.027095, 11.453964, 2.220004, 5.224014, -10.933884, 11.506168, 2.230000, 5.115729, -10.832931, 11.557304, 2.240012, 5.008284, -10.732018, 11.607445, 2.250007, 4.901773, -10.656068, 11.656440, 2.260014, 4.796209, -10.549095, 11.704435, 2.270015, 4.691522, -10.467658, 11.751355, 2.280008, 4.587804, -10.379812, 11.797198, 2.290005, 4.484979, -10.285004, 11.842037, 2.300006, 4.382968, -10.200114, 11.885871, 2.310008, 4.281773, -10.116996, 11.928699, 2.320010, 4.181398, -10.035723, 11.970520, 2.330009, 4.081845, -9.956324, 12.011334, 2.340003, 3.983117, -9.878847, 12.051141, 2.350009, 3.885123, -9.793789, 12.090015, 2.360006, 3.787865, -9.729069, 12.127881, 2.370011, 3.691339, -9.647362, 12.164814, 2.380003, 3.595549, -9.586630, 12.200741, 2.390000, 3.500494, -9.508202, 12.235736, 2.400001, 3.406073, -9.440999, 12.269801, 2.410005, 3.312285, -9.375461, 12.302935, 2.420010, 3.219125, -9.311562, 12.335142, 2.430014, 3.126591, -9.249266, 12.366422, 2.440017, 3.034682, -9.188577, 12.396776, 2.450016, 2.943396, -9.129456, 12.426208, 2.460010, 2.852731, -9.071905, 12.454718, 2.470024, 2.762568, -9.003763, 12.482382, 2.480003, 2.673025, -8.972992, 12.509057, 2.490028, 2.583970, -8.882798, 12.534962, 2.500015, 2.495405, -8.868087, 12.559884, 2.510020, 2.407445, -8.792147, 12.583969, 2.520011, 2.319964, -8.756067, 12.607147, 2.530019, 2.232949, -8.694473, 12.629495, 2.540011, 2.146398, -8.662026, 12.650942, 2.550020, 2.060298, -8.602377, 12.671563, 2.560009, 1.974646, -8.573774, 12.691289, 2.570016, 1.889431, -8.515960, 12.710196, 2.580000, 1.804649, -8.491484, 12.728214, 2.590001, 1.720289, -8.435412, 12.745418, 2.600020, 1.636164, -8.396494, 12.761811, 2.610013, 1.552438, -8.378650, 12.777324, 2.620024, 1.469096, -8.324768, 12.792032, 2.630004, 1.386142, -8.312121, 12.805866, 2.640002, 1.303561, -8.259537, 12.818899, 2.650023, 1.221114, -8.227927, 12.831135, 2.660005, 1.139020, -8.224095, 12.842505, 2.670009, 1.057260, -8.172433, 12.853082, 2.680041, 0.975547, -8.145100, 12.862869, 2.690026, 0.894149, -8.152411, 12.871797, 2.700040, 0.813041, -8.099712, 12.879938, 2.710092, 0.731856, -8.076027, 12.887295, 2.720084, 0.650940, -8.098660, 12.893799, 2.730119, 0.570256, -8.040253, 12.899521, 2.740064, 0.489898, -8.080140, 12.904394, 2.750052, 0.409849, -8.014529, 12.908487, 2.760118, 0.329418, -7.990427, 12.911803, 2.770014, 0.249435, -8.081909, 12.914272, 2.780524, 0.167739, -7.773512, 12.916034, 2.790295, 0.086588, -8.305218, 12.916880};
	
	public Map<Double, ArrayList<Double>> getPathMap(double[] pathArray) {
		Map<Double, ArrayList<Double>> pathmap = new HashMap<Double, ArrayList<Double>>();
		for(int i = 0; i < pathArray.length; i += 4) {
			ArrayList<Double> vel_pos = new ArrayList<Double>(2);
			vel_pos.add(Math.round(pathArray[i+1]*100.0)/100.0);
			vel_pos.add(Math.round(pathArray[i+2]*100.0)/100.0);
			pathmap.put(pathArray[i], vel_pos);
		}
		return pathmap;
		
	}
	ArrayList<Double> normalLeft = new ArrayList<Double>();
	ArrayList<Double> normalRight = new ArrayList<Double>();
	
	public void normalize() {
		for (int i = 0; i < pathLeft.length; i += 1) {
			normalLeft.add(pathLeft[i]);
		}
		for (int j = 0; j < pathRight.length; j += 1) {
			normalRight.add(pathRight[j]);
		}
	}
	
	@Override
	public void autonomousPeriodic() {
		robotdrive.drive(0, 0);
	}
	
	
	public void spline(char side) throws IOException {
		System.out.println("AGAIN");
		normalize();
		
		leftencoder.reset();
		rightencoder.reset();
		
		double minVolt = 0.2;
		double k1 = 1.0/(14*1.2);
		double k2 = 0.6;
		double k3 = 0.0;
		double leftSpeed = 0.0;
		double rightSpeed = 0.0;
		double leftError = 0.0;
		double rightError = 0.0;
		double leftAccel = 0.0;
		double rightAccel = 0.0;
		double TICKS_SCALE = 1.0/100.0;
		
		if(side == 'L') {
			normalLeft = reader.left;
			normalRight = reader.right;
		} else {
			normalRight = reader.left;
			normalLeft = reader.right;
		}
		
		double errorBetween = 0.0;
		double k4 = 0.00;
		for (int i = 1; i < normalLeft.size(); i += 8) {
			//System.out.println("Percent " + (i/(double)normalLeft.size()));
			//System.out.println("Encoders " + leftencoder.get() + " " + rightencoder.get());
			leftSpeed = normalLeft.get(i);
			rightSpeed = normalRight.get(i);
			//if (i <= normalLeft.size()-3) {
				leftError = normalLeft.get(i+2)-(double)leftencoder.get()*TICKS_SCALE;
				rightError = normalRight.get(i+2)-(double)rightencoder.get()*TICKS_SCALE;
			//}
			errorBetween = leftError - rightError;
			leftAccel = normalLeft.get(i+1);
			rightAccel = normalRight.get(i+1);
			
			Timer.delay(0.01);
			double leftVoltage = k1*leftSpeed+k2*leftError+k3*leftAccel-k4*errorBetween+minVolt;
			double rightVoltage = k1*rightSpeed+k2*rightError+k3*rightAccel+k4*errorBetween+minVolt;
			System.out.println("Target" + normalLeft.get(i+2) + " " + normalRight.get(i+2));
			System.out.println("Error: " + leftError + " " + rightError);
			robotdrive.tankDrive(leftVoltage, rightVoltage);
		}
		//}*/ 
		Timer.delay(0.05);
		robotdrive.tankDrive(0, 0);
	}

	
	static final double VOLT_DAMP = 0.00055;
	
	public void goForward(double distance, double maxSpeed) {
		leftencoder.reset();
		rightencoder.reset();
		
		double leftVolt = maxSpeed;
		double rightVolt = maxSpeed;
		
		double Ku = 0.25 * (0.0007/VOLT_DAMP); // In seconds
		double Tu = 0.25 * (0.0007/VOLT_DAMP); // In seconds
		double RAMP_UP_TICKS = 200;
		double MIN_SPEED_UP = 0.5;
		double MIN_SPEED_DOWN = 0.5;
		double volt = 0.0;
		double errorAccumulation = 0.0;
		double errorDamp = 1.25;
		double integral = 0.0;
		double I = (VOLT_DAMP/0.6) * 1.2 * (Ku/Tu);
		double error = 0.0;
		double prevError = 0.0;
		double prevTime = 0.0;
		double curTime = 1000.0*System.currentTimeMillis();
		double derivative = 0.0;
		double D = (VOLT_DAMP/0.6) * (3.0*Ku*Tu/40.0);

		while (leftencoder.get()/2 + rightencoder.get()/2 < distance) {
			robotdrive.tankDrive(leftVolt, rightVolt);
			double ticksTraveled = (leftencoder.get()/2 + rightencoder.get()/2);
			
			double encoderLeft = leftencoder.get();
			double encoderRight = rightencoder.get();
			SmartDashboard.putNumber("leftEncoders", encoderLeft);
			SmartDashboard.putNumber("rightEncoders", encoderRight);
			
			Timer.delay(0.01);
			
			prevError = error;
			error = encoderLeft-encoderRight;
			//if (integral*error < 0.0) integral = 0.0;
			integral += error*0.2;
			errorAccumulation += error*VOLT_DAMP*Ku;
			if (ticksTraveled > 0.0 && ticksTraveled < RAMP_UP_TICKS) {
				volt = MIN_SPEED_UP + (maxSpeed-MIN_SPEED_UP)*(ticksTraveled/RAMP_UP_TICKS);
			} else if (ticksTraveled >= distance - RAMP_UP_TICKS) {
				volt = MIN_SPEED_DOWN + (maxSpeed-MIN_SPEED_DOWN)*(distance-ticksTraveled)/(distance-RAMP_UP_TICKS);
			} else {
				volt = maxSpeed;
			}
			
			prevTime = curTime;
			curTime = 1000.0*System.currentTimeMillis();
			derivative = (error - prevError)/(curTime - prevTime);
			
			leftVolt = volt - errorAccumulation + I*integral + D*derivative;
			rightVolt = volt + errorAccumulation + I*integral + D*derivative;
		} 
		robotdrive.tankDrive(0, 0);
		return;
	}
	
	public void turnTest(double degrees, double speed, double rampUpDegrees) {
		ahrs.reset();
		ahrs.resetDisplacement();
		turn(degrees, speed, rampUpDegrees);
	}
	
	public void turn(double degrees, double speed, double rampUpDegrees) {
		System.out.println("turn");
		ahrs.reset();
		ahrs.resetDisplacement();
		
		
		double sign = degrees/Math.abs(degrees);
		double volt = 0.0;
		double MIN_SPEED_UP = 0.71;
		double MIN_SPEED_DOWN = 0.6;
		int timesRun = 0;
		System.out.println(ahrs.getAngle() + " " + degrees);
		System.out.println(Math.abs(ahrs.getAngle()) < Math.abs(degrees));
		double angle;
		ArrayList<Double> angleArray = new ArrayList<Double>();
		do {
			angle = 0;
			for(int a = 0; a < 10; a++) {
				angle += ahrs.getAngle();
			}
			angle /= 10.0;
			
			System.out.println("angle: " + angle);
			angleArray.add(ahrs.getAngle());
			timesRun++;
			SmartDashboard.putNumber("gyro", angle);
			if (Math.abs(angle) > 0.0 && Math.abs(angle) < Math.abs(rampUpDegrees)) {
				volt = (MIN_SPEED_UP + (speed-MIN_SPEED_UP)*(Math.abs(angle)/rampUpDegrees))*sign;
			} else if (Math.abs(angle) >= Math.abs(degrees) - rampUpDegrees) {
				volt = (MIN_SPEED_DOWN + (speed-MIN_SPEED_DOWN)*(Math.abs(degrees-angle))/(Math.abs(degrees)-rampUpDegrees))*sign;
			} else {
				volt = speed*sign;
			}
			SmartDashboard.putNumber("voltage", volt);
			System.out.println("voltage: " + volt);
			//if (!((Math.abs(ahrs.getAngle()) < Math.abs(degrees)))) System.out.println("angle: " + ahrs.getAngle());
			robotdrive.tankDrive(-volt, volt);
			System.out.println("angle after: " + angle);
			//if (!((Math.abs(ahrs.getAngle()) < Math.abs(degrees)))) System.out.println("angle: " + ahrs.getAngle());
		} while(Math.abs(angle) < Math.abs(2*degrees));
		int above70 = 0;
		System.out.println(angleArray.size());
		for (int i = 0; i < angleArray.size(); i++) {
			if (angleArray.get(i) >= 70.0) {
				System.out.println("get(i): " + angleArray.get(i) + " i: " + i);
				above70++;
			}
			if (above70 == 5) break;
		}
		System.out.println("times run: " + timesRun);
		System.out.println("done");
		volt = 0.0;
		
		robotdrive.tankDrive(0, 0);
	}

	@Override
	public void teleopInit() {
		// This makes sure that the autonomous stops running when
		// teleop starts running. If you want the autonomous to
		// continue until interrupted by another command, remove
		// this line or comment it out.
		leftencoder.reset();
		rightencoder.reset();
	}

	/**
	 * This function is called periodically during operator control
	 */
	double delay = 0.05;
	final double MIN_JOYSTICK_READ_STRAIGHT = 0.025; // Minimum registered axis movement from joystick. For forward/backward moving
	final double MIN_JOYSTICK_READ_TURN = 0.025; // Same as above but for turning
//	double correctionFactor = .4/400;
	double teleopMaxVoltage = 1.0;
	final double MAX_ENCODERS_PER_SECOND = 600.0;
	final double ACCELERATION_CONSTANT = 0.1;
	final double TANK_DRIVE_MAX = 0.75;
	double leftVolt = 0;
	double rightVolt = 0;

	double leftSpeed = 0.0;
	double rightSpeed = 0.0; 
	
	@Override
	public void teleopPeriodic() {
		
		/*
		// TANK DRIVE
		robotdrive.tankDrive(TANK_DRIVE_MAX*-joy.getRawAxis(5), TANK_DRIVE_MAX*-joy.getRawAxis(1));
		*/
		
		//if (Math.abs(joy.getRawAxis(5)) > 0.1) {
		double straightSpeed = -1*joy.getRawAxis(5);
		
		//if (Math.abs(joy.getRawAxis(0)) > 0.1) {
		double turnSpeed = joy.getRawAxis(0);
		
		//Ignores slight movement in turn joystick
		if (turnSpeed <= MIN_JOYSTICK_READ_TURN && turnSpeed >= -MIN_JOYSTICK_READ_TURN) {
			turnSpeed = 0.0;
		}
//		} else if (turnSpeed > MIN_JOYSTICK_READ_TURN) {
//			turnSpeed -= MIN_JOYSTICK_READ_TURN;
//		} else if (turnSpeed < -MIN_JOYSTICK_READ_TURN) {
//			turnSpeed += MIN_JOYSTICK_READ_TURN;
//		}
//		turnSpeed *= 1.0/(1.0 - MIN_JOYSTICK_READ_TURN);
		
		//Ignores slight movement in straight motion
		if (straightSpeed <= MIN_JOYSTICK_READ_STRAIGHT && straightSpeed >= -MIN_JOYSTICK_READ_STRAIGHT) {
			straightSpeed = 0.0;
		}
//		} else if (straightSpeed > MIN_JOYSTICK_READ_STRAIGHT) {
//			straightSpeed -= MIN_JOYSTICK_READ_STRAIGHT;
//		} else if (straightSpeed < -MIN_JOYSTICK_READ_STRAIGHT) {
//			straightSpeed += MIN_JOYSTICK_READ_STRAIGHT;
//		}
//		straightSpeed *= 1.0/(1.0 - MIN_JOYSTICK_READ_STRAIGHT);
		
//		double desiredLeft = (straightSpeed - 2 * turnSpeed)/(1+Math.abs(2 * turnSpeed));
//		double desiredRight = (straightSpeed + 2 * turnSpeed)/(1+Math.abs(2 * turnSpeed));
		double desiredLeft = (straightSpeed - turnSpeed)/(1+Math.abs(turnSpeed));
		double desiredRight = (straightSpeed + turnSpeed)/(1+Math.abs(turnSpeed));
		//if (Math.abs(desiredLeft) <= 0.15) desiredLeft = 0;
		//if (Math.abs(desiredRight) <= 0.15) desiredRight = 0;
		
		//System.out.println("Straight: " + straightSpeed + ", Turn: " + turnSpeed);
		
		double encoderLeftInitial = (double)leftencoder.get();
		double encoderRightInitial = (double)rightencoder.get();
		Timer.delay(delay);
		double encoderLeftFinal = (double)leftencoder.get();
		double encoderRightFinal = (double)rightencoder.get();
		
		double actualSpeedLeft = (encoderLeftFinal - encoderLeftInitial)/(delay*MAX_ENCODERS_PER_SECOND);
		double actualSpeedRight = (encoderRightFinal - encoderRightInitial)/(delay*MAX_ENCODERS_PER_SECOND);
		
		double leftError = desiredLeft - actualSpeedLeft;
		double rightError =  desiredRight - actualSpeedRight;
		double leftChange = ACCELERATION_CONSTANT * leftError;
		double rightChange = ACCELERATION_CONSTANT * rightError;
		
		leftSpeed += leftChange;
		rightSpeed += rightChange;
		
//		if (actualSpeedLeft < desiredLeft) {
//			leftVolt += leftChange;
//		} else if (actualSpeedLeft > desiredLeft) {
//			leftVolt -= leftChange;
//		} else {
//			leftVolt = desiredLeft;
//		}
//		
//		if (actualSpeedRight < desiredRight) {
//			rightVolt += rightChange;
//		} else if (actualSpeedRight > desiredRight) {
//			rightVolt -= rightChange;
//		} else {
//			rightVolt = desiredRight;
//		}
		
//		double minVolt = 0.2;
//		double k1 = 1.0/(14*1.2);
//		double k2 = 0.6;
//		double k3 = 0.0;
//		double leftSpeed = 0.0;
//		double rightSpeed = 0.0;
//		double leftAccel = 0.0;
//		double rightAccel = 0.0;
//		double TICKS_SCALE = 1.0/100.0;
//		
//		//System.out.println("Percent " + (i/(double)normalLeft.size()));
//		//System.out.println("Encoders " + leftencoder.get() + " " + rightencoder.get());
//		leftSpeed = desiredLeft;
//		rightSpeed = desiredRight;
//		//if (i <= normalLeft.size()-3) {
//		leftError = normalLeft.get(i+2)-(double)leftencoder.get()*TICKS_SCALE;
//		rightError = normalRight.get(i+2)-(double)rightencoder.get()*TICKS_SCALE;
//		//}
//		errorBetween = leftError - rightError;
//		leftAccel = normalLeft.get(i+1);
//		rightAccel = normalRight.get(i+1);
//		
//		Timer.delay(0.01);
//		double leftVoltage = k1*leftSpeed+k2*leftError+k3*leftAccel-k4*errorBetween+minVolt;
//		double rightVoltage = k1*rightSpeed+k2*rightError+k3*rightAccel+k4*errorBetween+minVolt;
//		System.out.println("Target" + normalLeft.get(i+2) + " " + normalRight.get(i+2));
//		System.out.println("Error: " + leftError + " " + rightError);
//		robotdrive.tankDrive(leftVoltage, rightVoltage);
		
//		if (Math.abs(desiredLeft) <= 0.1) desiredLeft = 0;
//		if (Math.abs(desiredRight) <= 0.1) desiredRight = 0;
		//robotdrive.tankDrive(teleopMaxVoltage*(Math.abs(leftVolt)*leftVolt), teleopMaxVoltage*(Math.abs(rightVolt)*rightVolt));

		SmartDashboard.putNumber("LeftEncoder: ", encoderLeftFinal);
		SmartDashboard.putNumber("RightEncoder: ", encoderRightFinal);
		SmartDashboard.putNumber("DesiredLeft: ", desiredLeft);
		SmartDashboard.putNumber("DesiredRight: ", desiredRight);
		SmartDashboard.putNumber("StraightSpeed: ", straightSpeed);
		SmartDashboard.putNumber("TurnSpeed: ", turnSpeed);
		SmartDashboard.putNumber("LeftError: ", leftError);
		SmartDashboard.putNumber("RightError: ", rightError);
		SmartDashboard.putNumber("Error Diffrence (right minus left): ", rightError - leftError);
		SmartDashboard.putNumber("leftSpeed: ", leftSpeed);
		SmartDashboard.putNumber("rightSpeed: ", rightSpeed);
		
		System.out.println("LeftEncoder: " + encoderLeftFinal + "\nRightEncoder: " + encoderRightFinal);
		System.out.println("DesiredLeft: " + desiredLeft + "\nDesiredRight: " + desiredRight + "\n");
		System.out.println("StraightSpeed: " + straightSpeed + "\nTurnSpeed: " + turnSpeed + "\n");
		System.out.println("LeftError: " + leftError + "\nRightError: " + rightError + "\n");
		
//		robotdrive.tankDrive(teleopMaxVoltage*desiredLeft, teleopMaxVoltage*desiredRight);
//		robotdrive.tankDrive(teleopMaxVoltage*leftVolt, teleopMaxVoltage*rightVolt);
		robotdrive.tankDrive(teleopMaxVoltage*leftSpeed, teleopMaxVoltage*rightSpeed);
	}

	/**
	 * This function is called periodically during test mode
	 */
	@Override
	public void testPeriodic() {
		LiveWindow.run();
	}
}
