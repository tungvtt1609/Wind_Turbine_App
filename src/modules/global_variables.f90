module global_variables
    ! This module contain the global variables that are used in this code
    ! Please declare all the global variables in this MODULE and USE it other files that you need the variable
    
    use user_defined_types

    implicit none

    !DEC$ IF .NOT. DEFINED(__LINUX__)
    !DEC$ attributes  DLLEXPORT :: pCtrlInputFile
    !DEC$ END IF
    

    ! Parameters
    logical  generator_cutin
    integer  PartialLoadControlMode, stoptype
    integer  CtrlStatus   ! Integer indicating the status of the controller.
                          ! (0: Normal operation, <0: Start-up, >0: Shutdown for different reasons)
    real(mk) deltat
    real(mk) RatedWindSpeed 
    real(mk) GenSpeedRefMax, GenSpeedRefMin, PeRated, GenTorqueRated, PitchStopAng, GenTorqueMax
    real(mk) TTfa_PWR_lower, TTfa_PWR_upper, TorqueCtrlRatio
    real(mk) Kopt, Kopt_dot, TSR_opt, R, GearRatio
    real(mk) Vcutout, Vstorm
    real(mk) Err0, ErrDot0, PitNonLin1, rel_limit
    integer  NAve_Pitch
    real(mk) PitchAngles(1000,3),PitchRefs(1000,3)
    real(mk) TAve_Pitch,DeltaPitchThreshold,AveragedMeanPitchAngles(3),AveragedPitchReference(3)

    ! Advanced controller input file
    character(256) :: additionalCtrlParamFilename = ''
    ! Dynamic variables
    integer :: stepno = 0, w_region = 0
    real(mk) AddedPitchRate, PitchColRef0, GenTorqueRef0
    real(mk) :: PitchColRef = 0.0_mk
    real(mk) :: GenTorqueRef = 0.0_mk
    real(mk) :: PitchColRefOld = 0.0_mk
    real(mk) :: GenTorqueRefOld = 0.0_mk
    real(mk) :: TimerGenCutin = 0.0_mk
    real(mk) :: TimerStartup = 0.0_mk
    real(mk) :: TimerExcl = 0.0_mk
    real(mk) :: TimerShutdown = 0.0_mk
    real(mk) :: TimerShutdown2 = 0.0_mk
    real(mk) GenSpeed_at_stop, GenTorque_at_stop
    real(mk) excl_flag
    real(mk) :: outmax_old=0.0_mk,outmin_old=0.0_mk
    logical :: fullload=.false.
    integer :: CountPitchDeviation=0
    real(mk) :: lambdaAtMinCt = 0.0_mk, pitchAtMinCt = 0.0_mk, minCt = 0.0_mk
    real(mk) :: CpAtMinCt = 0.0_mk 
	real(mk) :: GenTorque_addition = 0.0_mk
	real(mk) :: Pitch_addition = 0.0_mk
    !
    ! Variables with user defined Types
    !
    type(Tlowpass2order), save :: omega2ordervar
    type(Tlowpass2order), save :: power2ordervar
    type(Tfirstordervar), save :: pitchfirstordervar
    type(Tfirstordervar), save :: wspfirstordervar
    type(Tpidvar),        save :: PID_gen_var
    type(Tnotch2order),   save :: DT_mode_filt
    type(Tnotch2order),   save :: DT_mode_filt_torque
    type(Tnotch2order),   save :: pwr_DT_mode_filt
    type(Tpid2var),       save :: PID_pit_var
    type(Tdamper),        save :: DT_damper
    type(Tdamper),        save :: TTfa_damper
    type(Tfirstordervar), save :: TTfa_PWRfirstordervar
    type(Tcutin),         save :: CutinVar
    type(Tcutout),        save :: CutoutVar
    type(Tswitch),        save :: SwitchVar
    type(TSafetySystem),  save :: MoniVar
    type(TPitchGSvar),    save :: PitchGSVar
    type(TDeratevar),     save :: Deratevar
    type(TWindEstvar),    save :: WindEstvar
    type(Twpdata),        save :: OPdatavar
    type(Texclzone),      save :: ExcluZone
	!
	! Define variables for floating control
	type(TFloatingvar),   save :: Floatingvar
	type(Tfirstordervar), save :: switchingvar
	type(Tlowpass2order), save :: float2orderlpfvar
	type(Tbandpassfilt), save :: float2orderbpfvar

    ! defined global variables for down regulation control strategy
    type(TdownRegulationData), save :: downRegulationData
    type(TcontrolFile),        save :: additionalCtrlParamFile

    ! define global avariables for windspeed estimator
    type(TCpData),        save :: CpData

    type(TcontrolFile), pointer :: pCtrlInputFile

    ! define global avariables for external 3rd party DLLs
    Type (Tdll),          save :: external_dll
!**************************************************************************************************
end module global_variables
    
    
