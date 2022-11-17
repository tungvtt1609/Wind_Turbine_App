module dtu_we_controller_bladed

contains
!**************************************************************************************************
subroutine DISCON (avrSWAP, aviFAIL, avcINFILE, avcOUTNAME, avcMSG) bind(c,name='DISCON')

   use, intrinsic :: ISO_C_Binding
   use misc_mod
   use dtu_we_controller

   implicit none

   !DEC$ IF .NOT. DEFINED(__LINUX__)
   !DEC$ ATTRIBUTES DLLEXPORT :: DISCON 
   !GCC$ ATTRIBUTES DLLEXPORT :: DISCON 
   !DEC$ END IF
   
   
   ! Passed in Variables from simulation codes (OpenFAST or Bladed):
   real(c_float),   intent(inout)    :: avrSWAP    (*)   ! The swap array, used to send data to, and receive data from, the DLL controller.
   integer(c_int),  intent(inout)    :: aviFAIL          ! A flag used to indicate the success of this DLL call set as follows:
                                                         !        = 0 if the DLL call was successful.
                                                         !        > 0 if the DLL call was successful but cMessage should be issued as a warning messsage.
                                                         !        < 0 if the DLL call was unsuccessful or for any other reason the simulation
                                                         !            is to be stopped at this point with cMessage as the error message.

   character(Kind=c_char), intent(in   )  :: avcINFILE (nint(avrSWAP(50)))   ! An array of 1-byte CHARACTERs giving the name of the parameter input file, 'DISCON.IN'.

   character(Kind=c_char), intent(inout)  :: avcMSG    (nint(avrSWAP(49)))   ! An array of 1-byte CHARACTERS giving the message contained in cMessage,
                                                                             ! which will be displayed by the calling program if aviFAIL <> 0.

   character(Kind=c_char), intent(inout)  :: avcOUTNAME(nint(avrSWAP(51)))   ! An array of 1-byte CHARACTERS giving the simulation run name
                                                                             ! including path without extension.
   
   ! Define local Variables
   real(c_double) array1(100), array2(100)
   real(c_double),save :: gearboxRatio = 1.0
   integer(4) :: i, iostat, callno = 0
   integer(c_int) :: iStatus

   CHARACTER(LEN=SIZE(avcINFILE))     :: cInFile      ! CHARACTER string giving the name of the parameter input file, 'DISCON.IN'
   CHARACTER(LEN=SIZE(avcMSG))        :: cMessage     ! CHARACTER string giving a message that will be displayed by the calling program
                                                        ! if aviFAIL <> 0.
   CHARACTER(LEN=SIZE(avcOUTNAME))    :: cOutName     ! CHARACTER string giving the simulation run name without extension.
   
   
   
   iStatus = nint(avrSWAP(1))

   ! Convert c character arrays to Fortran CHARACTER strings:

   cOutName = transfer(avcOUTNAME(1:len(cOutName)),cOutName)
   i = index(cOutName, C_NULL_CHAR) - 1      ! Find the c NULL character at the end of cOutName, if it has then remove it
   if (i>0) cOutName = cOutName(1:i)
   
   cInFile = transfer(avcINFILE(1:len(cInFile)),cInFile)
   i = index(cInFile, C_NULL_CHAR) - 1       ! Find the c NULL character at the end of cInFile, if it has then remove it
   if (i>0) cInFile = cInFile(1:i)
   
   if (callno == 0 .and. iStatus == 0) then
   
       pCtrlInputFile=>null()
       if(.not. associated(pCtrlInputFile)) then
           allocate(pCtrlInputFile)
       endif

       pCtrlInputFile%name = trim(cInFile) 

       ! Get a free file unit id for additional control parameter input file
       call GetFreeFileUnitDllCall(pCtrlInputFile%fileID) 
       
       ! Open the control parameter input file for needed by Bladed/FAST

       if(fileExistsDllCall(trim(adjustl(pCtrlInputFile%name)))) then
           open(unit=pCtrlInputFile%fileID,file=trim(adjustl(pCtrlInputFile%name)),action='read')
           write(6,'(A)') ' Reading controller input parameters from file: '//trim(adjustl(pCtrlInputFile%name)) 
       else
           write(6,*) ' ERROR: FAST/BLADED interface Control input file: '//trim(adjustl(pCtrlInputFile%name))//' does not exist.'
           stop
       endif

       ! Call a function which reads the DISCON input file and convert is to 
       ! type2_dll input array1 style controller init data block

       call type2_dll_input(pCtrlInputFile,array1)
       call init_regulation_advanced(array1, array2)

       gearboxRatio = array1(82)

       write(*,*) 'Controller input parameters read successfully!'
       write(*,*) 'Current time: ',dble(avrSWAP( 2))

       ! Controller initialization is done and set callno to 1
       callno = 1
       array1 = 0.0_mk

   endif

   array1 = 0.0_mk
   ! If iStatus = 0 means OpenFAST/Bladed running at time = 0.00,
   ! The DTUWEC starts to run at the second time step, eg., time = delta_t
   ! Therefore, the iStatus should be bigger than 0
   if (( iStatus > 0 ) .and. ( aviFAIL >= 0 ) )  then

      array1( 1) = dble(nint(avrSWAP( 2)*1000.0_mk)/1000.0_mk)   !    1: general time
      
      array1( 2) = dble(avrSWAP(20))/gearboxRatio   !    2: For Bladed/OpenFAST: convert Generator speed to rotor speed; For HAWC2: constraint bearing1 shaft_rot 1 only 2
     ! array1( 2) = dble(avrSWAP(21))            !    2: constraint bearing1 shaft_rot 1 only 2 
      array1( 3) = dble(avrSWAP( 4))             !    3: constraint bearing2 pitch1 1 only 1
      array1( 4) = dble(avrSWAP(33))             !    4: constraint bearing2 pitch2 1 only 1
      array1( 5) = dble(avrSWAP(34))             !    5: constraint bearing2 pitch3 1 only 1
      array1( 6) = 0.0_mk                        !  6-8: wind free_wind 1 0.0 0.0 hub height
      array1( 7) = dble(avrSWAP(27))
      array1( 8) = 0.0_mk
      array1( 9) = dble(avrSWAP(15))             !    9: elec. power
      array1(10) = int(0)                        !   10: grid flag  ; [1=no grid,0=grid]
      array1(11) = dble(avrSWAP(53))             !   11: Tower top x-acceleration
      array1(12) = dble(avrSWAP(54))             !   12: Tower top y-acceleration
      array1(13) = 0.0_mk                        !   13: reserved variable for blade 1 pitch angle used by IPC
      array1(14) = 0.0_mk                        !   14: reserved variable for blade 2 pitch angle used by IPC
      array1(15) = 0.0_mk                        !   15: reserved variable for blade 3 pitch angle used by IPC
      
      call update_regulation(array1, array2)
      
      avrSWAP(35) = 1.0e0          ! Generator contactor status: 1=main (high speed) variable-speed generator
      avrSWAP(36) = array2(25)     ! Shaft brake status: 0=off
      avrSWAP(41) = 0.0e0          ! Demanded yaw actuator torque
      avrSWAP(42) = array2( 2)   ! Use the command angles of all blades if using individual pitch
      avrSWAP(43) = array2( 3)   ! "
      avrSWAP(44) = array2( 4)   ! "
      avrSWAP(45) = array2( 2)   ! Use the command angle of blade 1 if using collective pitch
      avrSWAP(46) = 0.0e0          ! Demanded pitch rate (Collective pitch)
      avrSWAP(47) = array2( 1)/gearboxRatio     ! Demanded generator torque
      avrSWAP(48) = 0.0e0          ! Demanded nacelle yaw rate
      avrSWAP(55) = 0.0e0          ! Pitch override: 0=yes
      avrSWAP(56) = 0.0e0          ! Torque override: 0=yes
      avrSWAP(65) = 0.0e0          ! Number of variables returned for logging
      avrSWAP(72) = 0.0e0          ! Generator start-up resistance
      avrSWAP(79) = 0.0e0          ! Request for loads: 0=none
      avrSWAP(80) = 0.0e0          ! Variable slip current status
      avrSWAP(81) = 0.0e0          ! Variable slip current demand
   endif 
   
   return
end subroutine DISCON

end module dtu_we_controller_bladed
