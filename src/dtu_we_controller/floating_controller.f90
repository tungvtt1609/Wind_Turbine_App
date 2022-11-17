module floating_controller_mod
	!
	! Module contains functions for floating controller used by DTU Wind Energy Controller
	!
	use user_defined_types
	use global_variables
	use misc_mod
	implicit none
!**************************************************************************************************
contains
!**************************************************************************************************
! (TO-DO)function tower_velocity_calcuation()
!
!
!
subroutine floating_controller(CtrlStatus, time, Floatingvar, TTfa_vel, wsp, Pitch_addition, GenTorque_addition,dump_array)
	implicit none
	real(mk) switchfactor, TTfa_vel_filt, y(2), WSPfilt, GSvar
	real(mk), intent(in) :: TTfa_vel, wsp , time
	real(mk), intent(out):: Pitch_addition, GenTorque_addition
	real(mk) :: region
	type(TFloatingvar), intent(inout) :: Floatingvar
	integer,  intent(in) :: CtrlStatus
	real(mk), intent(in) :: dump_array(50)
	!******************
	! switching 
	!******************
	switchingvar%tau = 5.0_mk ! turning on the floating loop gradually
	if (CtrlStatus.eq.0 .and. time .gt. Floatingvar%time_on) then
		switchfactor=1.d0
	else
		switchfactor=0.d0
	endif
	Floatingvar%switchfactor=lowpass1orderfilt(deltat,stepno,switchingvar,switchfactor) 
	!!******************
	! filtering tower signal
	!!! Low-pass filter
	if (float2orderlpfvar%f0 .gt. 0) then
		y = lowpass2orderfilt(deltat, stepno, float2orderlpfvar, TTfa_vel)   
	else
		y = TTfa_vel
	endif
	!!! Band-pass filter 
	if (float2orderbpfvar%f0 .gt.0) then
		y = bandpassfilt(deltat, stepno, float2orderbpfvar, TTfa_vel)
	else 
		y = TTfa_vel
	endif
	TTfa_vel_filt = y(1)
	!!******************
	! filterd wind speed for gain-scheduling

	if (Floatingvar%GSmode .gt. 0.0_mk)  then
		Floatingvar%GSvar = PitchColRef
	else
		WSPfilt = lowpass1orderfilt(deltat, stepno, wspfirstordervar, wsp)
		Floatingvar%GSvar = abs(WSPfilt - Floatingvar%RatedWindSpeed)
	endif

	!!****************** 
	! Main Loop
	! Tower-pitch loop
	if ((abs(Floatingvar%TPgain) .gt. 0.0_mk).and.(time.gt.20.0_mk)) then
			region = switchfactor*dump_array(10)
			call towerV_pitch(Pitch_addition,TTfa_vel_filt*region,Floatingvar)  ! only active in above-rated
	endif
 ! Tower-genTorq loop
	if ((abs(Floatingvar%TGgain) .gt. 0.0_mk).and.(time.gt.20.0_mk)) then
		region = switchfactor*dump_array(10)
		call towerV_genTorq(GenTorque_addition,TTfa_vel_filt*region,Floatingvar)
	endif
	! for output
	Floatingvar%switchfactor = switchfactor
	Floatingvar%TTfa_vel_filt = TTfa_vel_filt
end subroutine floating_controller
!
! additional loop maping the tower fore-aft velocity to pitch
subroutine towerV_pitch(Pitch_addition,TTfa_vel,Floatingvar)
	implicit none 
	real(mk), intent(in) :: TTfa_vel
	real(mk), intent(inout) :: Pitch_addition
	type(TFloatingvar), intent(in) :: Floatingvar
	real(mk) :: GSvar, GSvalue
	! main
	GSvar = Floatingvar%GSvar
	GSvalue = 1+Floatingvar%KK1_tp*GSvar + Floatingvar%KK2_tp*GSvar**2
	Pitch_addition = Floatingvar%TPgain*GSvalue*TTfa_vel
	return
end subroutine towerV_pitch
! additional loop maping the tower fore-aft velocity to genTorque
subroutine towerV_genTorq(GenTorque_addition,TTfa_vel,Floatingvar)
	implicit none 
	real(mk), intent(in) :: TTfa_vel
	real(mk), intent(inout) :: GenTorque_addition
	type(TFloatingvar), intent(in) :: Floatingvar
	real(mk) :: GSvar, GSvalue
	! main
	GSvar = Floatingvar%GSvar
	GSvalue = 1+Floatingvar%KK1_tq*GSvar + Floatingvar%KK2_tq*GSvar**2
	GenTorque_addition = Floatingvar%TGgain *GSvalue * TTfa_vel
	return
end subroutine towerV_genTorq
end module floating_controller_mod
