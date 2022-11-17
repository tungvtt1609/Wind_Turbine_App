module misc_mod
   !
   ! Module where IO and filter functions are defined.
   ! some parameters of the type defined filters have default values but not all
   !
   use user_defined_types

   implicit none

   integer, save :: ctrl_logfile_unit = 78

contains
!**************************************************************************************************
subroutine getFreeFileUnit(unitnr) bind(c, name='getFreeFileUnit')
    integer,intent(inout) :: unitnr
    integer, save :: lastUsedUnitNum = 127

    unitnr=lastUsedUnitNum+1
    lastUsedUnitNum=unitnr
end subroutine getFreeFileUnit

!**************************************************************************************************
subroutine GetFreeFileUnitDllCall(unitnr) bind(c, name='GetFreeFileUnitDllCall')
   !DEC$ IF .NOT. DEFINED(__LINUX__)
   !DEC$ ATTRIBUTES DLLEXPORT :: GetFreeFileUnitDllCall
   !GCC$ ATTRIBUTES DLLEXPORT :: GetFreeFileUnitDllCall
   !DEC$ END IF

    integer,intent(inout) :: unitnr
    integer, save :: lastUsedUnitNum = 127

    unitnr=lastUsedUnitNum+1
    lastUsedUnitNum=unitnr
end subroutine GetFreeFileUnitDllCall


!**************************************************************************************************
! Returns true if a file or folder exist
logical function fileExists(filename)
    character(len=*),intent(in) :: filename  ! 
    INQUIRE(FILE=trim(filename), EXIST=fileExists) ! true if exist
end function fileExists

!**************************************************************************************************
! Returns true if a file or folder exist
! logical function fileExistsDllCall(filename) bind(c, name='fileExistsDllCall')
!    !DEC$ IF .NOT. DEFINED(__LINUX__)
!    !DEC$ ATTRIBUTES DLLEXPORT :: fileExistsDllCall
!    !GCC$ ATTRIBUTES DLLEXPORT :: fileExistsDllCall
!    !DEC$ END IF
!     character(len=*),intent(in) :: filename  ! 
!     INQUIRE(FILE=trim(filename), EXIST=fileExistsDllCall) ! true if exist
! end function fileExistsDllCall

!**************************************************************************************************
! Returns true if a file or folder exist
 logical function fileExistsDllCall(filename)
    !!DEC$ IF .NOT. DEFINED(__LINUX__)
    !!DEC$ ATTRIBUTES DLLEXPORT :: fileExistsDllCall
    !!GCC$ ATTRIBUTES DLLEXPORT :: fileExistsDllCall
    !!DEC$ END IF
     character(len=*),intent(in) :: filename  ! 
     INQUIRE(FILE=trim(filename), EXIST=fileExistsDllCall) ! true if exist
 end function fileExistsDllCall


!**************************************************************************************************
function lowpass1orderfilt(dt, stepno, filt, x)
   ! First order low-pass filter.
   integer stepno
   real(mk) lowpass1orderfilt, dt, x, y, a1, b1, b0, tau
   type(Tfirstordervar) filt
   ! Step
   if ((stepno .eq. 1) .and. (stepno .gt. filt%stepno1)) then
      filt%x1_old = x
      filt%y1_old = x
      y=x
   else
      if (stepno .gt. filt%stepno1) then
         filt%x1_old = filt%x1
         filt%y1_old = filt%y1
      endif
      tau = filt%tau
      a1 = (2.0_mk*tau - dt) / (2.0_mk*tau + dt)
      b0 = dt / (2.0_mk*tau + dt)
      b1 = b0
      y = a1*filt%y1_old + b0*x + b1*filt%x1_old
   endif
   ! Save previous values
   filt%x1 = x
   filt%y1 = y
   filt%stepno1 = stepno
   ! Output
   lowpass1orderfilt = y
   return
end function lowpass1orderfilt
!**************************************************************************************************
function lowpass2orderfilt(dt, stepno, filt, x)

   ! Second order low-pass filter.
   real(mk) lowpass2orderfilt(2), dt, x
   integer stepno
   type(Tlowpass2order) filt
   ! local vars
   real(mk) y, f0, zeta, a1, a2, b0, b1, b2, denom
   ! Step
   if ((stepno .eq. 1) .and. (stepno .gt. filt%stepno1)) then
     filt%x1 = x
     filt%x2 = x
     filt%x1_old = filt%x1
     filt%x2_old = filt%x2
     filt%y1 = x
     filt%y2 = x
     filt%y1_old = filt%y1
     filt%y2_old = filt%y2
     y=x
   else
     if (stepno .gt. filt%stepno1) then
       filt%x1_old = filt%x1
       filt%x2_old = filt%x2
       filt%y1_old = filt%y1
       filt%y2_old = filt%y2
     endif
     f0 = filt%f0
     zeta = filt%zeta
     denom = 3.0_mk + 6.0_mk*zeta*pi*f0*dt + 4.0_mk*pi**2*f0**2*dt**2
     a1 = (6.0_mk - 4.0_mk*pi**2*f0**2*dt**2)/denom
     a2 = (-3.0_mk + 6.0_mk*zeta*pi*f0*dt - 4.0_mk*pi**2*f0**2*dt**2)/denom
     b0 = 4.0_mk*pi**2*f0**2*dt**2/denom
     b1 = b0
     b2 = b0
     y = a1*filt%y1_old + a2*filt%y2_old + b0*x + b1*filt%x1_old + b2*filt%x2_old
   endif
   ! Save previous values
   filt%x2 = filt%x1_old
   filt%x1 = x
   filt%y2 = filt%y1_old
   filt%y1 = y
   filt%stepno1=stepno
   ! Output
   lowpass2orderfilt(1) = y
   lowpass2orderfilt(2) = (y - filt%y1_old)/dt
   return
end function lowpass2orderfilt
!**************************************************************************************************
function notch2orderfilt(dt,stepno,filt,x)
   ! Second order notch filter.
   real(mk) notch2orderfilt,dt,x
   integer stepno
   type(Tnotch2order) filt
   ! local vars
   real(mk) y, f0, zeta1, zeta2, a1, a2, b0, b1, b2, denom
   ! Step
   if ((stepno .eq. 1) .and. (stepno .gt. filt%stepno1)) then
      filt%x1 = x
      filt%x2 = x
      filt%x1_old = filt%x1
      filt%x2_old = filt%x2
      filt%y1 = x
      filt%y2 = x
      filt%y1_old = filt%y1
      filt%y2_old = filt%y2
      y = x
   else
      if (stepno .gt. filt%stepno1) then
         filt%x1_old = filt%x1
         filt%x2_old = filt%x2
         filt%y1_old = filt%y1
         filt%y2_old = filt%y2
      endif
      f0 = filt%f0
      zeta1 = filt%zeta1
      zeta2 = filt%zeta2
      denom = 3.0_mk + 6.0_mk*zeta1*pi*f0*dt + 4.0_mk*pi**2*f0**2*dt**2
      a1 = ( 6.0_mk - 4.0_mk*pi**2*f0**2*dt**2)/denom
      a2 = (-3.0_mk + 6.0_mk*zeta1*pi*f0*dt - 4.0_mk*pi**2*f0**2*dt**2)/denom
      b0 = ( 3.0_mk + 6.0_mk*zeta2*pi*f0*dt + 4.0_mk*pi**2*f0**2*dt**2)/denom
      b1 = (-6.0_mk + 4.0_mk*pi**2*f0**2*dt**2)/denom
      b2 = ( 3.0_mk - 6.0_mk*zeta2*pi*f0*dt + 4.0_mk*pi**2*f0**2*dt**2)/denom
      y = a1*filt%y1_old + a2*filt%y2_old + b0*x + b1*filt%x1_old + b2*filt%x2_old
   endif
   ! Save previous values
   filt%x2 = filt%x1_old
   filt%x1 = x
   filt%y2 = filt%y1_old
   filt%y1 = y
   filt%stepno1 = stepno
   ! Output
   notch2orderfilt = y
   return
end function notch2orderfilt
!**************************************************************************************************
function bandpassfilt(dt, stepno, filt, x)
   ! Second order band-pass filter.
   real(mk) bandpassfilt, dt, x
   integer stepno
   type(Tbandpassfilt) filt
   ! local vars
   real(mk) y,f0,zeta,tau,a1,a2,b0,b1,b2,denom
   ! Step
   if ((stepno .eq. 1) .and. (stepno .gt. filt%stepno1)) then
      filt%x1 = x
      filt%x2 = x
      filt%x1_old = filt%x1
      filt%x2_old = filt%x2
      filt%y1 = x
      filt%y2 = x
      filt%y1_old = filt%y1
      filt%y2_old = filt%y2
      y = x
   else
      if (stepno .gt. filt%stepno1) then
         filt%x1_old = filt%x1
         filt%x2_old = filt%x2
         filt%y1_old = filt%y1
         filt%y2_old = filt%y2
      endif
      f0 = filt%f0
      zeta = filt%zeta
      tau = filt%tau
      denom = 3.0_mk + 6.0_mk*zeta*pi*f0*dt + 4.0_mk*pi**2*f0**2*dt**2
      a1 = -(-6.0_mk + 4.0_mk*pi**2*f0**2*dt**2)/denom
      a2 = -( 3.0_mk - 6.0_mk*zeta*pi*f0*dt + 4.0_mk*pi**2*f0**2*dt**2)/denom
      b0 = -(-6.0_mk*zeta*pi*f0*dt - 12.0_mk*zeta*pi*f0*tau)/denom
      b1 =  -24.0_mk*zeta*pi*f0*tau/denom
      b2 = -( 6.0_mk*zeta*pi*f0*dt - 12.0_mk*zeta*pi*f0*tau)/denom
      y = a1*filt%y1_old + a2*filt%y2_old + b0*x + b1*filt%x1_old + b2*filt%x2_old
   endif
   ! Save previous values
   filt%x2 = filt%x1_old
   filt%x1 = x
   filt%y2 = filt%y1_old
   filt%y1 = y
   filt%stepno1 = stepno
   ! Output
   bandpassfilt = y
   return
end function bandpassfilt
!**************************************************************************************************
function timedelay(dt, stepno, filt, Td, x)
   ! Time delay.
   integer stepno, n, k
   real(mk) x, timedelay, dt, Td
   type(Ttdelay) filt
   n = nint(Td/dt)
   ! Step
   if ((stepno .eq. 1)) then
      do k = 1, 40
         filt%xz(k) = x
      end do
   endif
   if (stepno .gt. filt%stepno1) then
      filt%xz_old = filt%xz
   endif
   do k = 40, 2, -1
       filt%xz(k) = filt%xz_old(k - 1)
   end do
   filt%xz(1) = x
   ! Output
   if (Td .eq. 0.0_mk) then
      timedelay = x
   else
      timedelay = filt%xz(n)
   endif
   filt%stepno1 = stepno
   return
end function timedelay
!**************************************************************************************************
end module misc_mod
