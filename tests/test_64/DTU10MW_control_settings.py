import numpy as np

settings_dict =    {1: 10000.0,    	# Rated power [kW]                         
         2:   0.628,    	# Minimum rotor (LSS) speed [rad/s]
         3:   1.005 ,   	# Rated rotor (LSS) speed [rad/s]
         4:  15.6E+06 , 	# Maximum allowable generator torque [Nm]
         5:  0.0 ,   	# Minimum pitch angle, theta_min [deg], !NOTE THIS WAS CHANGED to 0.0 FOR THE PYTHON TEST
								# if |theta_min|>90, then a table of <wsp,theta_min> is read #
								# from a file named 'wptable.n', where n=int(theta_min)
         6:  82.0,    	# Maximum pitch angle [deg]
         7:  10.0,    	# Maximum pitch velocity operation [deg/s]
         8:   0.4,    	# Frequency of generator speed filter [Hz]
         9:   0.7,    	# Damping ratio of speed filter [-]
        10:   1.80,   	# Frequency of free-free DT torsion mode [Hz], if zero no notch filter used
      # Partial load control parameters
        11:   13013100.0, # Optimal Cp tracking K factor [Nm/(rad/s)^2], #
                                # Qg=K*Omega^2, K=eta*0.5*rho*A*Cp_opt*R^3/lambda_opt^3                     
        12:   0.683456e8, # Proportional gain of torque controller [Nm/(rad/s)]
        13:   0.153367e8, # Integral gain of torque controller [Nm/rad]
        14:   0.0,    	# Differential gain of torque controller [Nm/(rad/s^2)]
#     Full load control parameters
        15:   1   ,   	# Generator control switch [1=constant power, 0=constant torque]
        16:   1.06713,    # Proportional gain of pitch controller [rad/(rad/s)]
        17:   0.242445,   # Integral gain of pitch controller [rad/rad]
        18:   0.0,    	# Differential gain of pitch controller [rad/(rad/s^2)]
        19:   0.4e-8, 	# Proportional power error gain [rad/W]
        20:   0.4e-8, 	# Integral power error gain [rad/(Ws)]
        21: 	 11.4,       # Coefficient of linear term in aerodynamic gain scheduling, KK1 [deg]
        22: 	 402.9,      # Coefficient of quadratic term in aerodynamic gain scheduling, KK2 [deg^2] &
								# (if zero, KK1 = pitch angle at double gain)
        23:   1.3,    	# Relative speed for double nonlinear gain [-]
#     Cut-in simulation parameters
        24:  -1,    # Cut-in time [s], no cut-in is simulated if zero or negative
        25:  1.0,   # Time delay for soft start of torque [1/1P]
#     Cut-out simulation parameters
        26:  -1,    # Shut-down time [s], no shut-down is simulated if zero or negative
        27:   5.0,  # Time of linear torque cut-out during a generator assisted stop [s]
        28:  1,     # Stop type [1=normal, 2=emergency]
        29:  1.0,   # Time delay for pitch stop after shut-down signal [s]
        30:  3,     # Maximum pitch velocity during initial period of stop [deg/s]
        31:  3.0,   # Time period of initial pitch stop phase [s] (maintains pitch speed specified in constant 30)
        32:  4,     # Maximum pitch velocity during final phase of stop [deg/s]
#     Expert parameters (keep default values unless otherwise given)
        33:   2.0,  	# Time for the maximum torque rate = Maximum allowable generator torque/(constant 33 + 0.01s) [s]
        34:   2.0,  	# Upper angle above lowest minimum pitch angle for switch [deg], if equal then hard switch
        35:  95.0,  	# Percentage of the rated speed when the torque limits are fully opened [%]
        36:   2.0,  	# Time constant of 1st order filter on wind speed used for minimum pitch [1/1P]
        37:   1.0,  	# Time constant of 1st order filter on pitch angle used for gain scheduling [1/1P]
#     Drivetrain damper
        38:   0.0,  	# Proportional gain of active DT damper [Nm/(rad/s)], requires frequency in input 10
#	  Over speed
	    39:  25.0,  	# Overspeed percentage before initiating turbine controller alarm (shut-down) [%]
#     Additional non-linear pitch control term (not used when all zero)
	    40:   0.0,  	# Rotor speed error scaling factor [rad/s]
	    41:   0.0,  	# Rotor acceleration error scaling factor [rad/s^2]
	    42:   0.0,  	# Pitch rate gain [rad/s]
#     Storm control command
	   43:   28.0,  	# Wind speed 'Vstorm' above which derating of rotor speed is used [m/s]
	   44:   28.0,  	# Cut-out wind speed (only used for derating of rotor speed in storm) [m/s]	  
#     Safety system parameters
	   45:   30.0,  # Overspeed percentage before initiating safety system alarm (shut-down) [%]
	   46:    1.5,  # Max low-pass filtered tower top acceleration level [m/s^2]
#     Turbine parameter
	   47:  178.0,  # Nominal rotor diameter [m]
#     Parameters for rotor inertia reduction in variable speed region
       48:    0.0,  # Proportional gain on rotor acceleration in variable speed region [Nm/(rad/s^2)] (not used when zero)
#     Parameters for alternative partial load controller with PI regulated TSR tracking
       49:    7.8,  # Optimal tip speed ratio [-] (only used when K=constant 11 = 0 otherwise  Qg=K*Omega^2 is used)
#     Parameters for adding aerodynamic drivetrain damping on gain scheduling
       50:    0.0,  # Aerodynamic DT damping coefficient at the operational point of zero pitch angle [Nm/(rad/s)] (not used when zero)
       51:    0.0,  # Coefficient of linear term in aerodynamic DT damping scheduling, KK1 [deg]
       52:    0.0,  # Coefficient of quadratic term in aerodynamic DT damping scheduling, KK2 [deg^2]
#     Torque exclusion zone
       53:     0.0, # Exclusion zone: Lower speed limit [rad/s] (Default 0 used if zero)	  
       54:     0.0, # Exclusion zone: Generator torque at lower limit [Nm] (Default 0 used if zero)	  
       55:     0.0, # Exclusion zone: Upper speed limit [rad/s] (if =< 0 then exclusion zone functionality is inactive)               	  
       56:     0.0, # Exclusion zone: Generator torque at upper limit [Nm] (Default 0 used if zero) 	  
       57:     0.0, # Time constant of reference switching at exclusion zone [s] (Default 0 used if zero)	  
#     DT torsion mode damper	  
       58:     0.0, # Frequency of notch filter [Hz] (Default 10 x input 10 used if zero)	  
       59:     0.0, # Damping of BP filter [-] (Default 0.02 used if zero) 	  
       60:     0.0, # Damping of notch filter [-] (Default 0.01 used if zero) 	  
       61:     0.0, # Phase lag of damper [s] =>  max 40*dt (Default 0 used if zero) 	  
#     Fore-aft Tower mode damper	  
       62:     0.0, # Frequency of BP filter [Hz] (Default 10 used if zero)\\ 	  
       63:     0.0, # Frequency of notch fiter [Hz] (Default 10 used if zero)\\ 	  
       64:     0.0, # Damping of BP filter [-] (Default 0.02 used if zero)\\	  
       65:     0.0, # Damping of notch filter [-] (Default 0.01 used if zero)\\	  
       66:     0.0, # Gain of damper [-] (Default 0 used if zero)\\ 	  
       67:     0.0, # Phase lag of damper [s] =>  max 40*dt (Default 0 used if zero)\\ 	  
       68:     0.0, # Time constant of 1st order filter on PWR used for fore-aft Tower mode damper GS [Hz] (Default 10 used if zero)	  
       69:     0.0, # Lower PWR limit used for fore-aft Tower mode damper GS [-] (Default 0 used if zero)	  
       70:     0.0, # Upper PWR limit used for fore-aft Tower mode damper GS [-] (Default 0 used if zero) 	  
#     Side-to-side Tower mode filter	  
       71:     0.0, # Frequency of Tower side-to-sede notch filter [Hz] (Default 100 used if zero)	  
       72:     0.0, # Damping of notch filter [-] (Default 0.01 used if zero)	  
       73:     0.0, # Max low-pass filtered tower top acceleration level before initiating safety system alarm (shut-down) [m/s^2] (Default 1.1 x input 46 used if zero)	  
       74:     0.0, # Time constant of 1st order filter on tower top acceleration [1/1P] (Default 1 used if zero)	  
#     Pitch deviation monitor parameters	  
       75: 1005020, # Parameters for pitch deviation monitoring. The format is 1,nnn,mmm 	  
                          # where 'nnn' [s] is the period of the moving average and 'mmm' is threshold of the deviation [0.1 deg] (functionality is inactive if value $<$ 1,000,000)	  
#     Gear ratio	  
       76:     0.0} # Gear ratio used for the calculation of the LSS rotational speeds and the HSS generator torque reference [-] (Default 1 if zero)	 
      
      
settings = np.zeros(100)

for k, v in settings_dict.items():
    settings[k-1] = v
#print(settings)
