module cyclic_pitch_controller_mod
use BuildInfo
use cyclic_pitch_controller_fcns_mod
implicit none
contains
!**************************************************************************************************
subroutine init_cyclic_pitch_controller(array1,array2) bind(c,name='init_cyclic_pitch_controller')
! Applies perturbation to control signals
  implicit none
      !DEC$ IF .NOT. DEFINED(__LINUX__)
      !DEC$ ATTRIBUTES DLLEXPORT :: init_cyclic_pitch_controller
      !GCC$ ATTRIBUTES DLLEXPORT :: init_cyclic_pitch_controller
      !DEC$ END IF
  real*8 array1(20),array2(1)
  ! Input parameters
  !   1: constant   1  ; Lead angle [deg]
  !   2: constant   2  ; Proportional gain at zero pitch [deg/MNm]
  !   3: constant   3  ; Integral gain at zero pitch [deg/(MNm*s)]  
  !   4: constant   4  ; Differential gain at zero pitch [deg*s/MNm]  
  !   5: constant   5  ; Coefficient of linear term in aerodynamic gain scheduling, KK1 [deg]
  !   6: constant   6  ; Coefficient of quadratic term in aerodynamic gain scheduling, KK2 [deg^2] 
  !                    ; (if zero, KK1 = pitch angle at double gain)
  !   7: constant   7  ; Low-pass filter frequency [Hz]
  !   8: constant   8  ; Low-pass filter damping ratio [-]
  !   9: constant   9  ; Low-pass filter time constant for gain scheduling [s]
  !  10: constant  10  ; Maximum amplitude on cyclic pitch [deg]
  !  11: constant  11  ; Thresshold for full power switch [-]
  !  12: constant  12  ; Start time for activting the ipc [s]

  ! write the dll version info to the command windows
  call echo_version()

  ! get values from input array1
  LeadAngle        = array1(1)*degrad
  PID_cos_var%Kpro = array1(2)*1.d-6*degrad
  PID_cos_var%Kint = array1(3)*1.d-6*degrad
  PID_cos_var%Kdif = array1(4)*1.d-6*degrad
  invkk1=1.d0/(array1(5)*degrad)
  if (array1(6).eq.0.d0) then
    invkk2=0.d0
  else
    invkk2=1.d0/(array1(6)*degrad*degrad)
  endif
  LP2_cos_var%f0   = array1(7) 
  LP2_cos_var%zeta = array1(8)
  LP1_pit_var%tau  = array1(9)
  PID_cos_var%outmin =-array1(10)*degrad
  PID_cos_var%outmax = array1(10)*degrad
  thr_ratcyclic = array1(11)
  if(array1(12) .gt. 50.0) then 
    time_on       = array1(12)
  endif 
  ! The two PID controllers are the same
  LP2_sin_var = LP2_cos_var
  PID_sin_var = PID_cos_var
  LP1_rpm_var = LP1_pit_var
  ! Initialize the transformation matrices
  call setup_mbc_matrices
  ! Switch filter
  LP1_status_var%tau=10.d0
  ! dummy array
  array2 = 0.d0
  return
end subroutine init_cyclic_pitch_controller
!**************************************************************************************************
subroutine update_cyclic_pitch_controller(array1,array2) bind(c,name='update_cyclic_pitch_controller')
implicit none
    !DEC$ IF .NOT. DEFINED(__LINUX__)
    !DEC$ ATTRIBUTES DLLEXPORT :: update_cyclic_pitch_controller
    !GCC$ ATTRIBUTES DLLEXPORT :: update_cyclic_pitch_controller
    !DEC$ END IF

real*8 array1(50),array2(20)
! Input array1 must contains
!
!    1: general time [s]     
!    2: Azimuth angle of blade 1 (zero = blade up) [rad]
!    3: Rotor speed [rad/s]
!    4: Flap BRM of blade 1 (pos. bend. forward) [kNm]    
!    5: Flap BRM of blade 2 (pos. bend. forward) [kNm]    
!    6: Flap BRM of blade 3 (pos. bend. forward) [kNm]    
!    7: Pitch angle reference of blade 1 from collective pitch controller [rad]    
!    8: Pitch angle reference of blade 2 from collective pitch controller [rad]    
!    9: Pitch angle reference of blade 3 from collective pitch controller [rad]    
!   10: Status flag from collective pitch controller [0=normal operation]
!   11: Pullpower switch from main ctrl: dll inpvec 1 14 
!
! Output array2 contains
!
!    1: Pitch angle reference of blade 1      [rad]  without mean cpc anlge
!    2: Pitch angle reference of blade 2      [rad]  without mean cpc anlge 
!    3: Pitch angle reference of blade 3      [rad]  without mean cpc anlge 
!    4: Cosine moment                         [kNm]    
!    5: Sine moment                           [kNm]    
!    6: Filtered cosine moment                [kNm]    
!    7: Filtered sine moment                  [kNm]    
!    8: Cosine pitch                          [rad]    
!    9: Sine pitch                            [rad]    
!   10: Switch factor                         [ - ]
!
! Local variables
integer*4 i,CtrlStatus
real*8 time,AziAng,y(2),momvec_rot(3),momvec_mbc(3)
real*8 pitref(3),meanpitref,meanpitreffilt,kgain(3)
real*8 momcos_filt,momsin_filt,pitref_mbc(3)
real*8 omega,omegafilt
real*8 switchfactor,r_switch_filt

! Input
time  =array1(1)
AziAng=array1(2) ! in rad [-pi,+pi]
omega =array1(3)
momvec_rot=-array1(3+rev_blade_no)*1.d3 ! [Nm] Flap BRMs with sign that leads to positive pitch angles for positive moments bending the blade downwind
pitref=array1(6+rev_blade_no) 
CtrlStatus=int(array1(10))
! Specify the time for switch on individual pitch
if (CtrlStatus.eq.0 .and. time .gt. time_on) then
  switchfactor=1.d0
else
  switchfactor=0.d0
endif
! active the ipc when the ctrl switch value from main dll is larger than thr_ratcyclic
if (array1(11).gt.thr_ratcyclic) then
   r_switch_filt = 1.d0
else
   r_switch_filt = 0.d0
endif
switchfactor=lowpass1orderfilt(deltat,stepno,LP1_status_var,r_switch_filt*switchfactor)

! Increment time step (may actually not be necessary in type2 DLLs)
if (time-time_old.gt.1.d-6) then
  deltat=time-time_old
  time_old=time
  stepno=stepno+1
endif

! Reference pitch angles and its low-pass filter for gain scheduling
meanpitref=(pitref(1)+pitref(2)+pitref(3))/3.d0
meanpitreffilt=lowpass1orderfilt(deltat,stepno,LP1_pit_var,meanpitref)
kgain=1.d0/(1.d0+meanpitreffilt*invkk1+meanpitreffilt**2*invkk2)

! Low-pass filtered rotor speed
omegafilt=lowpass1orderfilt(deltat,stepno,LP1_rpm_var,omega)
NP_cos_var%f0=omegafilt/2.d0/pi
NP_sin_var%f0=omegafilt/2.d0/pi

! Inverse Coleman transform to get non-rotating moments
momvec_mbc=matmul(InvBmat(AziAng),momvec_rot)

! Low-pass filter these asymmetric moments
y=lowpass2orderfilt(deltat,stepno,LP2_cos_var,momvec_mbc(2))
momcos_filt=y(1)
y=lowpass2orderfilt(deltat,stepno,LP2_sin_var,momvec_mbc(3))
momsin_filt=y(1)

! Notch filter these low-pass filtered asymmetric moments
momcos_filt=notch2orderfilt(deltat,stepno,NP_cos_var,momcos_filt)
momsin_filt=notch2orderfilt(deltat,stepno,NP_sin_var,momsin_filt)

! Set collective pitch to mean and calculate the PID feedback
pitref_mbc(1)=meanpitref
pitref_mbc(2)=PID(stepno,deltat,kgain,PID_cos_var,momcos_filt)
pitref_mbc(3)=PID(stepno,deltat,kgain,PID_sin_var,momsin_filt)

! Coleman transform including the lead angle to get the rotating pitch references
! ToDO: Do we need to gain schedule the lead anlge?
array2(rev_blade_no) = switchfactor*(matmul(Bmat(AziAng+LeadAngle),pitref_mbc)-pitref)
!array2(rev_blade_no)=pitref+switchfactor*(matmul(Bmat(AziAng+LeadAngle),pitref_mbc)-pitref)
array2(4:5)=momvec_mbc(2:3)*1.d-3    ! [kNm]
array2(6)  =momcos_filt*1.d-3        ! [kNm]
array2(7)  =momsin_filt*1.d-3        ! [kNm]
array2(8:9)=pitref_mbc(2:3)
array2(10) =switchfactor
return
end subroutine update_cyclic_pitch_controller
!**************************************************************************************************
end module cyclic_pitch_controller_mod
