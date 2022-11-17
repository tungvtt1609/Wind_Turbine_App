module user_defined_types 

    use global_constants
    use iso_c_binding

    implicit none 
    ! =============================================================================
    ! User Defined Types for Filters
    ! =============================================================================
    
    ! First order filter
    type Tfirstordervar
       real(mk) tau, x1, x1_old, y1, y1_old
       integer :: stepno1 = 0
    end type Tfirstordervar
    !  Second order low pass filter filter
    type Tlowpass2order
       real(mk) zeta, f0, x1, x2, x1_old, x2_old, y1, y2, y1_old, y2_old
       integer :: stepno1 = 0
    end type Tlowpass2order
    !  Second order notch filter
    type Tnotch2order
       real(mk) :: zeta1 = 0.1_mk
       real(mk) :: zeta2 = 0.001_mk
       real(mk) f0, x1, x2, x1_old, x2_old, y1, y2, y1_old, y2_old
       integer :: stepno1 = 0
    end type Tnotch2order
    !  Second order band-pass filter
    type Tbandpassfilt
       real(mk) :: zeta = 0.02_mk
       real(mk) :: tau = 0.0_mk
       real(mk) f0, x1, x2, x1_old, x2_old, y1, y2, y1_old, y2_old
       integer :: stepno1 = 0
    end type Tbandpassfilt
    !  Time delay
    type Ttdelay
       real(mk) xz(40)
       real(mk) xz_old(40)
       integer :: stepno1 = 0
    end type Ttdelay

    ! =============================================================================
    ! User Define Types for torque PID and Pitch PID
    ! =============================================================================

    ! Generator Torque PID data type
    type Tpidvar
       real(mk) Kpro,Kdif,Kint,outmin,outmax,velmax,error1,outset1,outres1
       integer :: stepno1 = 0
       real(mk) outset,outpro,outdif,error1_old,outset1_old,outres1_old,outres
    end type Tpidvar

    ! Blade pitch PID data type
    type Tpid2var
       real(mk) Kpro(2),Kdif(2),Kint(2),outmin,outmax,velmax,error1(2),outset1,outres1, Kpro_init(2)
       integer :: stepno1 = 0
       real(mk) outset,outpro,outdif,error1_old(2),outset1_old,outres1_old,outres
    end type Tpid2var

    ! Gain scheduling
    type TPitchGSvar
       real(mk) invkk1, invkk2
       real(mk) kp_speed, invkk1_speed, invkk2_speed
    end type

    ! Wind speed vs pitch angle data type
    type Twpdata
       real(mk) wpdata(MAXWPLINES,2)
       integer lines
    end type Twpdata

    ! Partial load and full load region switch data type
    type Tswitch
       real(mk) pitang_lower, pitang_upper, rel_sp_open_Qg
    end type

    ! =============================================================================
    ! User defined types for advanced features
    ! =============================================================================

    ! Damper data type
    type Tdamper
       type(Tbandpassfilt) bandpass
       type(Tnotch2order) notch
       type(Ttdelay) delay
       real(mk) gain, Td
    end type

    ! rotor speed exclusion zone data type
    type Texclzone
       type(Tnotch2order) notch
       real(mk) Lwr, Lwr_Tg, Hwr, Hwr_Tg, time_excl_delay
    end type

    ! De-rating control data type
    ! TODO: modification is needed
    type TDeratevar
       integer  :: strat          
       real(mk) :: dr = 1.0_mk
    end type

    ! Wind speed estimator data type
    type TWindEstvar
        real(mk) :: J, est_Qa , sum_err, xhat ,P , Q , R, Kp, Ki, radius
    end type
	
	! Floating
	type TFloatingvar
		real(mk) :: TPgain = 0.0_mk
		real(mk) ::	TGgain = 0.0_mk
		real(mk) :: time_on = 0.0_mk
		real(mk) :: KK1_tp = 0.0_mk ! gain-scheduling for tower-pitch loop
		real(mk) :: KK2_tp = 0.0_mk
		real(mk) :: KK1_tq = 0.0_mk ! gain-scheduling for tower-genTorq loop
		real(mk) :: KK2_tq = 0.0_mk 
		real(mk) :: RatedWindSpeed = 11.4_mk 
		real(mk) :: GSmode = 1.0_mk
		real(mk) :: GSvar = 0.0_mk
		real(mk) :: switchfactor = 0.0_mk
		real(mk) :: TTfa_vel_filt = 0.0_mk
	end type

    ! MIN CT control data type
    type TdownRegulationData
        real(mk) Cp(MAX_NUM_Pitch,1)
        real(mk) Ct(MAX_NUM_Pitch,1)
        real(mk) Lambda(MAX_NUM_Lambda)
        real(mk) Pitch(MAX_NUM_Pitch)
        real(mk) dCp(100)
        integer :: NumLines
        integer :: NumColumns
        character(256) :: fileName=''
    end type

    ! Cp-lambda data type
    type TCpData
        real(mk), dimension(:,:), allocatable :: Cp
        ! real(mk) Lambda(MAX_NUM_Lambda)
        real(mk) Lambda(3)
        !real(mk) Pitch(MAX_NUM_Pitch)
        real(mk) Pitch(3)
        integer :: NumPitchAngles
        integer :: NumLambdas
        character(256) :: fileName=''
        real(mk), dimension(:,:), allocatable :: LambdaList
        real(mk), dimension(:,:), allocatable :: PitchList
    end type

    ! Cut in data type
    type Tcutin
       real(mk) time, delay
    end type

    ! Cut out data type
    type Tcutout
       integer stoptype
       real(mk) time, pitchdelay, pitchdelay2, torquedelay, pitchvelmax, pitchvelmax2
    end type

    ! Safe system data type
    type TSafetySystem
       real(mk) overspeed, RysteVagtLevel
       type(Tlowpass2order) omega2ordervar
       type(Tfirstordervar) rystevagtfirstordervar
    end type

    ! Reserved data type for IO
    type TcontrolOutputFile
        character(len=256) :: name
        character(len=256) :: subFileName 
        integer            :: fileID,subFileID,lineNumber=0, subFileLineNumber = 0
    end type TcontrolOutputFile

    ! Additional controller input file data type for IO
    type TcontrolFile
        character(len=256) :: name
        character(len=256) :: includedFileName
        integer            :: fileID,lineNumber=0
    end type TcontrolFile

    ! IO data type
    type Tline
        character*512 :: line 
    end type Tline

    ! IO data type
    type Tword
        character(len=WORD_LENGTH),dimension(MAX_WORDS) :: wordArray
        integer :: numWords
    end type Tword

    type Tstring
        character(len=:),allocatable :: str
        integer :: length
    end type Tstring

    ! DLL data type
    Type Tdll
        integer (C_INTPTR_T)         :: p_dll                 ! integer pointer to DLL
#if defined __GFORTRAN__
        type(C_FUNPTR)         :: p_func                ! integer pointer to DLL subroutine discon
#elif defined __INTEL_COMPILER
        integer (C_INTPTR_T)         :: p_func                ! integer pointer to DLL subroutine discon
#endif

        character*256                :: filename=''
        character*256				 :: infile =''
        character*256				 :: outfile=''
        character*256				 :: func_name=''
        character*256				 :: par_filename=''
        real(4),dimension(32)        :: avrSWAP=0.0          ! The swap array, used to pass data to, and receive data from, the DLL controller.
        integer(c_int)               :: is_shutdown
        integer(c_int)               :: hawc2_status
        integer                      :: istep_hawc2 
        integer                      :: istep_controller    !
        real*8					     :: aero_power
        real*8					     :: elec_power
        real*8                       :: efficiency
        real*8                       :: time_step = 0.d0     ! time step used in simulation [s]
    end Type Tdll

end module user_defined_types