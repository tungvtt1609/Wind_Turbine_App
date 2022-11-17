module dtu_we_controller_fcns
   !
   ! Module with general functions that are used by the DTU Wind Energy Controller. 
   !
   use user_defined_types
   use global_variables
   use misc_mod
   implicit none
!**************************************************************************************************
contains
!**************************************************************************************************
function switch_spline(x, x0, x1)
   ! A function that goes from 0 at x0 to 1 at x1
   real(mk) switch_spline, x, x0, x1
   if (x0 .ge. x1) then
      if (x .lt. x0) then
         switch_spline = 0.0_mk
      else
         switch_spline = 1.0_mk
      endif
   else
      if (x .lt. x0) then
         switch_spline = 0.0_mk
      elseif (x .gt. x1) then
         switch_spline = 1.0_mk
      else
         switch_spline = (x - x0)/(x1 - x0)
      endif
   endif
   return
end function switch_spline
!**************************************************************************************************
function interpolate(x, x0, x1, f0, f1)
   ! Linear interpolation of x through the points (x0, f0) and (x1, f1)
   real(mk) interpolate, x, x0, x1, f0, f1
   if (x0 .eq. x1) then
      interpolate = f0
   else
      interpolate = (x - x1)/(x0 - x1)*f0 + (x - x0)/(x1 - x0)*f1
   endif
   return
end function interpolate
!**************************************************************************************************
function GetOptiPitch(wsp)
   ! Computes pitch angle from look-up table based on wind speed input
   real(mk) GetOptiPitch,wsp
   ! local vars
   real(mk) x, x0, x1, f0, f1, pitch
   integer i
   i=1
   do while((OPdatavar%wpdata(i, 1) .le. wsp) .and. (i .le. OPdatavar%lines))
      i=i+1
   enddo
   if (i.eq.1) then
      GetOptiPitch = OPdatavar%wpdata(1, 2)
   elseif (i .gt. OPdatavar%lines) then
      GetOptiPitch = OPdatavar%wpdata(OPdatavar%lines, 2)
   else
      x = wsp
      x0 = OPdatavar%wpdata(i-1, 1)
      x1 = OPdatavar%wpdata(i, 1)
      f0 = OPdatavar%wpdata(i-1, 2)
      f1 = OPdatavar%wpdata(i, 2)
      Pitch = interpolate(x, x0, x1, f0, f1)
      GetOptiPitch = Pitch
   endif
   return
end function GetOptiPitch
!**************************************************************************************************
function int2str(i) result(res)
    character(:),allocatable :: res
    integer,intent(in) :: i
    character(range(i)+2) :: tmp
    write(tmp,'(i2.2)') i
    res = trim(tmp)
end function
!**************************************************************************************************
function real2str(r) result(res)
    character(:),allocatable :: res
    real(mk),intent(in) :: r
    character(range(r)+2) :: tmp
    write(tmp,'(F0.1)') r
    res = trim(tmp)
end function
!**************************************************************************************************

function PID(stepno, dt, kgain, PIDvar, error)
   ! PID controller with one input. Used for generator torque controller and
   ! tower fore-aft damper
   integer stepno
   real(mk) PID, dt, kgain(3), error
   type(Tpidvar) PIDvar
   ! Local vars
   real(mk) eps
   parameter(eps = 0.000001_mk)
   ! Initiate
   if (stepno.eq.1) then
      PIDvar%outset1 = 0.0_mk
      PIDvar%outres1 = 0.0_mk
      PIDvar%error1 = 0.0_mk
      PIDvar%error1_old = 0.0_mk
      PIDvar%outset1_old = 0.0_mk
      PIDvar%outres1_old = 0.0_mk
   endif
   ! Save previous values
   if (stepno.gt.PIDvar%stepno1) then
      PIDvar%outset1_old = PIDvar%outset1
      PIDvar%outres1_old = PIDvar%outres1
      PIDvar%error1_old = PIDvar%error1
   endif
   ! Update the integral term
   PIDvar%outset = PIDvar%outset1_old + 0.5_mk*(error + PIDvar%error1_old)*Kgain(2)*PIDvar%Kint*dt
   ! Update proportional term
   PIDvar%outpro = Kgain(1)*PIDvar%Kpro*0.5_mk*(error + PIDvar%error1_old)
   ! Update differential term
   PIDvar%outdif = Kgain(3)*PIDvar%Kdif*(error - PIDvar%error1_old)/dt
   ! Sum to up
   PIDvar%outres = PIDvar%outset+PIDvar%outpro + PIDvar%outdif
   ! Satisfy hard limits
   if (PIDvar%outres .lt. PIDvar%outmin) then
      PIDvar%outres = PIDvar%outmin
   elseif (PIDvar%outres .gt. PIDvar%outmax) then
      PIDvar%outres = PIDvar%outmax
   endif
   ! Satisfy max velocity
   if (PIDvar%velmax .gt. eps) then
      if ((abs(PIDvar%outres-PIDvar%outres1_old)/dt) .gt. PIDvar%velmax) then
         PIDvar%outres = PIDvar%outres1_old + dsign(PIDvar%velmax*dt, PIDvar%outres-PIDvar%outres1_old)
      endif
   endif
   ! Anti-windup on integral term and save results
   PIDvar%outset1 = PIDvar%outres - PIDvar%outpro - PIDvar%outdif
   PIDvar%outres1 = PIDvar%outres
   PIDvar%error1 = error
   PIDvar%stepno1 = stepno
   ! Set output
   if (stepno .eq. 0) then
      PID = 0.0_mk
   else
      PID = PIDvar%outres
   endif
   return
end function PID
!**************************************************************************************************
function PID2(stepno,dt,kgain,PIDvar,error,added_term)
   ! PID controller with two inputs. Used for the pitch angle with feebacks from generator speed
   ! and power errors.
   integer stepno
   real(mk) PID2, dt, kgain(3, 2), error(2), added_term
   type(Tpid2var) PIDvar
   ! Local vars
   real(mk) eps
   parameter(eps=0.000001_mk)
   ! Initiate
   if (stepno .eq. 1) then
      PIDvar%outset1 = 0.0_mk
      PIDvar%outres1 = 0.0_mk
      PIDvar%error1 = 0.0_mk
      PIDvar%error1_old = 0.0_mk
      PIDvar%outset1_old = 0.0_mk
      PIDvar%outres1_old = 0.0_mk
   endif
   ! Save previous values
   if (stepno .gt. PIDvar%stepno1) then
      PIDvar%outset1_old = PIDvar%outset1
      PIDvar%outres1_old = PIDvar%outres1
      PIDvar%error1_old = PIDvar%error1
   endif
   ! Update the integral term
   PIDvar%outset = PIDvar%outset1_old + 0.5_mk*dt*(Kgain(2, 1)*PIDvar%Kint(1)*(error(1) + PIDvar%error1_old(1))&
                                                  +Kgain(2, 2)*PIDvar%Kint(2)*(error(2) + PIDvar%error1_old(2)))
   ! Update proportional term
   PIDvar%outpro = 0.5_mk*(Kgain(1, 1)*PIDvar%Kpro(1)*(error(1) + PIDvar%error1_old(1))&
                          +Kgain(1, 2)*PIDvar%Kpro(2)*(error(2) + PIDvar%error1_old(2)))
   ! Update differential term
   PIDvar%outdif = (Kgain(3, 1)*PIDvar%Kdif(1)*(error(1) - PIDvar%error1_old(1)))/dt + added_term*dt
   ! Sum up
   PIDvar%outres = PIDvar%outset + PIDvar%outpro + PIDvar%outdif
   ! Satisfy hard limits
   if (PIDvar%outres .lt. PIDvar%outmin) then
      PIDvar%outres = PIDvar%outmin
   elseif (PIDvar%outres .gt. PIDvar%outmax) then
      PIDvar%outres = PIDvar%outmax
   endif
   ! Write out values if the output is not-a-number
   ! NOTE: ebra: isnan is not standard
   if (isnan(PIDvar%outres)) then
      write(*, *)  'NaN issue. Stop in DTU controller'
      write(*, *)  'PIDvar%outres=', PIDvar%outres
      write(*, *)  'PIDvar%outpro=', PIDvar%outpro
      write(*, *)  'PIDvar%outdif=', PIDvar%outdif
      write(*, *)  'dt=', dt
      write(*, *)  'PIDvar%error1_old(1)=', PIDvar%error1_old(1)
      write(*, *)  'error(1)=', error(1)
      write(*, *)  'PIDvar%Kdif(1)=', PIDvar%Kdif(1)
      write(*, *)  'Padded_term=', added_term
      write(*, *)  'PIDvar%outset=', PIDvar%outset
      stop
   endif
   ! Satisfy max velocity
   if (PIDvar%velmax .gt. eps) then
      if ((abs(PIDvar%outres - PIDvar%outres1_old)/dt) .gt. PIDvar%velmax) then
         PIDvar%outres = PIDvar%outres1_old + dsign(PIDvar%velmax*dt, PIDvar%outres-PIDvar%outres1_old)
      endif
   endif
   ! Anti-windup on integral term and save results
   PIDvar%outset1 = PIDvar%outres - PIDvar%outpro - PIDvar%outdif
   PIDvar%outres1 = PIDvar%outres
   PIDvar%error1 = error
   PIDvar%stepno1 = stepno
   ! Set output
   if (stepno .eq. 0) then
      PID2 = 0.0_mk
   else
      PID2 = PIDvar%outres
   endif
   return
end function PID2

!**************************************************************************************************
subroutine damper(stepno, dt, x, filters, y, x_filt)
   ! General damper based on bandpass filter, notch filter, and a time delay.
   type(Tdamper), intent(in) :: filters
   integer stepno
   real(mk), intent(in)  :: x
   real(mk), intent(out) :: y, x_filt
   real(mk) dt
   x_filt = bandpassfilt(dt, stepno, filters%bandpass, x)
   x_filt = notch2orderfilt(dt, stepno, filters%notch, x_filt)
   x_filt = timedelay(dt, stepno, filters%delay, filters%Td, x_filt)
   y = filters%gain*x_filt
end subroutine damper
!**************************************************************************************************

!**************************************************************************************************
subroutine damper_twr(stepno, dt, x, filters, y, x_filt)
   ! Tower fore-aft damper based on notch filter, and a gain.
   type(Tdamper), intent(in) :: filters
   integer stepno
   real(mk), intent(in)  :: x
   real(mk), intent(out) :: y, x_filt
   real(mk) dt
  ! x_filt = bandpassfilt(dt, stepno, filters%bandpass, x)
   x_filt = notch2orderfilt(dt, stepno, filters%notch, x)
  ! x_filt = timedelay(dt, stepno, filters%delay, filters%Td, x_filt)
   y = filters%gain*x_filt
end subroutine damper_twr
!**************************************************************************************************


subroutine writeErrorMsg(controlDataFile,message)
    Type (TcontrolFile), pointer    :: controlDataFile
    character*(*), dimension(3), intent(in) :: message

    write(6,'(A,I3,A)') trim(message(1))//' ',controlDataFile%lineNumber,' '//trim(message(2))//trim(controlDataFile%name)
end subroutine writeErrorMsg

!**************************************************************************************************
! 1: file does not exist, 2: open file error, 3: wrong number of columns
subroutine errorCpAtMinCtTable(controlDataFile,errCode)
    type (TcontrolFile), pointer    :: controlDataFile
    ! type(Tstring), dimension(2) :: message
    integer,    intent(in) :: errCode
    ! message(1)%str = ' ERROR: Invalid command in file: '
    ! message(2)%str = ' at line: '

    if(errCode == 1) then
        write(6,'(A)') 'ERROR: Does not exist table file: '//trim(adjustl(controlDataFile%includedFileName))
    elseif(errCode == 2) then
        write(6,'(A)') 'ERROR: Could not open table file: '//trim(adjustl(controlDataFile%includedFileName))
    elseif(errCode == 3) then
        write(6,'(A)') 'ERROR: Wrong number of colums in table file: '//trim(adjustl(controlDataFile%includedFileName))
    endif
end subroutine errorCpAtMinCtTable

!**************************************************************************************************
subroutine errorInvalidCommand(controlDataFile)
    type (TcontrolFile), pointer    :: controlDataFile
    type(Tstring), dimension(2) :: message
    message(1)%str = ' ERROR: Invalid command in file: '
    message(2)%str = ' at line: '

    write(6,'(A,I3)') trim(message(1)%str)//trim(controlDataFile%name)//trim(message(2)%str),controlDataFile%lineNumber
end subroutine errorInvalidCommand

!**************************************************************************************************

subroutine errorInvalidBlock(controlDataFile)
    type (TcontrolFile), pointer    :: controlDataFile
    type(Tstring), dimension(2) :: message
    message(1)%str = ' ERROR: Invalid block name in file: '
    message(2)%str = ' at line: '
    write(6,'(A,I3)') trim(message(1)%str)//trim(controlDataFile%name)//trim(message(2)%str),controlDataFile%lineNumber
end subroutine errorInvalidBlock

!**************************************************************************************************

subroutine errorInvalidKeyword(controlDataFile)
    type (TcontrolFile), pointer    :: controlDataFile
    type(Tstring), dimension(2) :: message
    message(1)%str = ' ERROR: Invalid Keyword in file: '
    message(2)%str = ' at line: '
    write(6,'(A,I3)') trim(message(1)%str)//trim(controlDataFile%name)//trim(message(2)%str),controlDataFile%lineNumber
end subroutine errorInvalidKeyword

!**************************************************************************************************
subroutine errorInvalidParameter(controlDataFile)
    type (TcontrolFile), pointer    :: controlDataFile
    type(Tstring), dimension(2) :: message
    message(1)%str = ' ERROR: Invalid number of parameters in file: '
    message(2)%str = ' at command line: '
    write(6,'(A,I3)') trim(message(1)%str)//trim(controlDataFile%name)//trim(message(2)%str),controlDataFile%lineNumber
end subroutine errorInvalidParameter

!**************************************************************************************************

subroutine readline(controlDataFile,line)
    ! Declare subroution input argument variables
    Type (TcontrolFile),pointer    :: controlDataFile
    Type (Tline),intent(inout)     :: line

    ! read one line from the additional control parameters file   
    read(unit=controlDataFile%fileID,fmt='(A)') line
    controlDataFile%lineNumber = controlDataFile%lineNumber+1
    return
end subroutine readline

!**************************************************************************************************
subroutine getParameters(words,nstart,npars,out_vec,err)
    ! Declare subroution input argument variables
    type(Tword), intent(in)             :: words
    integer , intent(in)                :: nstart, &
                                           npars
    real(mk), dimension(:), intent(inout) :: out_vec
    logical, intent(inout)                :: err


    ! Declare loacl variables
    integer :: i
    
    ! Start the main part of this subroutine 
    if (words%numWords /= (nstart+npars-1)) then
        err=.true.
    else
        do i=nstart,nstart+npars-1
            read(words%wordArray(i),Fmt=*,err=99) out_vec(i-nstart+1)
            err=.false.
        enddo
    endif
    return
99  err = .true.
end subroutine getParameters

!**************************************************************************************************
! Divides a line into several words spaced by blank and #'s and @'s,line ended with ;
!**************************************************************************************************
subroutine getwords(line,words,controlDataFile)
    ! Declare subroution input argument variables
    character*(*),intent(inout)     :: line
    type (Tword), intent(inout)     :: words
    type (TcontrolFile),pointer     :: controlDataFile 

    ! Declare the local variables.
    character, parameter :: linestop =';', &
                            blank  = ' ',  &
                              tab  ='	', &
                            comma  = ',',  &
                            label ='#',    &
                            id   = '@'

    character(len=WORD_LENGTH), dimension(3) :: strMsg

    integer :: first, &
               last, &
		       endp, &
		       blankpos, &
		       tabpos, &
		       commapos, &
		       labelpos, &
		       idpos

    integer :: tmp_blank
    logical :: out
    ! integer :: error
    logical :: cont
    integer :: ivec4(4)
    integer :: sep_type

    cont=.false.

    do while (cont.eqv..false.)
        cont=.true.
        words%wordArray(:)=''
        line=trim(adjustl(line))
        endp=index(line,linestop)
        first=1
        words%numWords=0
        out=.false.

        if (endp>1) then
            do while (out.eqv..false.)
                blankpos=index(line(first:endp),blank)
	            tabpos  =index(line(first:endp),tab)
	            commapos=index(line(first:endp),comma)
	            labelpos=index(line(first:endp),label)
	            idpos=index(line(first:endp),id)
	            if (idpos.gt.0) then
                    continue
	            endif
	            if (commapos/=0) then
                    strMsg(1) = 'WARNING: A comma "," is in the command line: '
                    strMsg(2) = 'in File: '
                    call writeErrorMsg(controlDataFile,strMsg)
                endif

	            if ((tabpos/=0).or.(labelpos/=0).or.(idpos/=0)) then
                    ivec4=[blankpos,tabpos,labelpos,idpos]
                    blankpos=minval(ivec4,dim=1,mask=ivec4.gt.0)
                    sep_type=minloc(ivec4,dim=1,mask=ivec4.gt.0)
                else
                    sep_type=1
	            endif
            !   If the restline is without blanks, the restline is stored and
            !   the division in words is stopped ...
                if (blankpos==0) then
                    words%numWords=words%numWords+1
                    if (words%numWords>size(words%wordArray)) then 
                        write(6,*) 'WARNING: More than ',size(words%wordArray),' words in line. Exceeding words ignored'
                    endif
                    words%wordArray(words%numWords)=line(first:endp-1)
                    out=.true.
                    exit
                endif
                if (blankpos.eq.1) then
                    continue
                endif
            !   If a blank is found a word is stored ...
                if ((blankpos>1).or.((blankpos.eq.1).and.(sep_type.ge.3))) then
                    words%numWords=words%numWords+1
                    if (words%numWords>size(words%wordArray)) then
                        write(6,*) 'WARNING: More than ',size(words%wordArray),' words in line. Exceeding words ignored'
                    endif
                    if (sep_type.le.2) then
                        last=first+blankpos-2
                    else
                        last=max(first+blankpos-2,first)
                    endif     
                    words%wordArray(words%numWords)=line(first:last)
                    first=first+blankpos
                endif

            !   If search has reached and end of line or if too many words have
            !   been found then stop
                if (first>=endp) then
                    out=.true.
                    exit
	            endif
	            if (words%numWords>size(words%wordArray)) then
                    write(6,*) 'WARNING: More than ',size(words%wordArray),' words in line. Exceeding words ignored'
                endif

                blankpos=index(line(first:endp),blank)
	            tabpos  =index(line(first:endp),tab)
	            labelpos=index(line(first:endp),label)
	            idpos=index(line(first:endp),id)
                ivec4=[blankpos,tabpos,labelpos,idpos]
                tmp_blank=minval(ivec4,dim=1,mask=ivec4.gt.0)
                sep_type=minloc(ivec4,dim=1,mask=ivec4.gt.0)

                do while ((tmp_blank==1).and.(sep_type.le.2))
                    first=first+1
                    if (first>=endp) then
                        out=.true.
                        exit
                    endif
                    blankpos=index(line(first:endp),blank)
                    tabpos  =index(line(first:endp),tab)
                    labelpos=index(line(first:endp),label)
                    idpos=index(line(first:endp),id)
                    ivec4=[blankpos,tabpos,labelpos,idpos]
                    tmp_blank=minval(ivec4,dim=1,mask=ivec4.gt.0)
                enddo
            enddo
        elseif (endp==0) then 
            write(6,*) 'WARNING: No line termination symbol','[',linestop,'] in command line ',controlDataFile%lineNumber
        endif

    enddo

    return
end subroutine getwords

! **************************************************************************************************
 !subroutine type2_dll_input(ctrl_input_file,outvec)  bind(c, name='type2_dll_input')
  subroutine type2_dll_input(ctrl_input_file,outvec)  
     !!DEC$ IF .NOT. DEFINED(__LINUX__)
     !!DEC$ ATTRIBUTES DLLEXPORT :: type2_dll_input
     !!GCC$ ATTRIBUTES DLLEXPORT :: type2_dll_input
     !!DEC$ END IF
     ! passed in variables
     type(TcontrolFile),     pointer :: ctrl_input_file
     Real*8, dimension(:), intent(out) :: outvec
    
     ! loacl variables
     Type (Tline) :: line
     Type (Tword) :: words
     real*8, dimension(MAX_WORDS) :: rea
     
     integer :: i
     
     logical :: eof, &
                err, &
                all_ok
     eof =.false.
     outvec = 0.0_mk
     
     do while (eof .eqv..false.)
       call readline(ctrl_input_file,line)                 ! line is read
         call getwords(line%line,words,ctrl_input_file)         ! words in line is retrieved
         select case (trim(words%wordArray(1)))
         
         case ('constant')
           call getParameters(words,2,2,rea,err)
           if (err.eqv..true.) exit
     	  i=nint(rea(1))
     	  if (i.gt.(size(outvec))) then
     	    write(0,*) '*** ERROR *** In type2 dll initialize, number of channels mismatch with used number'
     	    write(0,*) '*** ERROR *** Error in command line ',ctrl_input_file%lineNumber,' Control input file name: ',trim(ctrl_input_file%name)
     	    stop 1
     	  endif
           outvec(i)=rea(2)
     
         case ('')
           continue
         case (';')
           continue
         case ('end')
           eof =.true.
           ! Control of data given above is OK
           all_ok=.true.
     !      if (all_ok.eqv..true.) then
     !        write(0,*) 'output commands read with succes'
     !      else
     !        write(0,*) '*** ERROR *** Not all needed output commands present - error'
     !      endif
     
     !      write(0,*) 'Output commands read'
     
         case default
           write(0,*) '*** ERROR *** Error in command line ',ctrl_input_file%lineNumber,' Control input file name: ',trim(ctrl_input_file%name)
         end select
     enddo
 end subroutine type2_dll_input

!**************************************************************************************************
subroutine readAdditionalCtrlParameter(controlDataFile,downRegulationData,Cptable,err)
    ! Declare subroution input argument variables
    type(TcontrolFile),     pointer :: controlDataFile
    type(TdownRegulationData), intent(inout)::downRegulationData
    logical, intent(inout)  ::  err

    ! Declare the local variables.
    type(TCpData) :: Cptable 
    type(Tline) :: line
    type(Tword) :: words
    character(len=16) :: errMsg
    integer     ::  errCode, i
    logical :: all_ok, &
    !           jumpOut, &
               eof

    ! Start main part of this subroutine.
    eof = .false.
    do while (eof .eqv..false.)
        call readline(controlDataFile,line)                 ! line is read
        call getwords(line%line,words,controlDataFile)      ! words in line is retrieved
        select case (trim(words%wordArray(1)))
        !-----------------------------------------------------------------
        case ('')
            continue
        !-----------------------------------------------------------------
        case (';')  ! this case may not be needed
            continue
        !-----------------------------------------------------------------
        case ('begin')
            !---------------------------------------------
            select case (trim(words%wordArray(2)))    
            case ('down_regulation')
                ! Rend the down_regulation block
                write(6,*) 'Reading the down_regulation block ...'
                call readDownRegulation(controlDataFile,downRegulationData,err,errMsg,errCode)
                if((err .eqv. .true.) .and. (trim(errMsg) == 'block')) then
                    call errorInvalidBlock(controlDataFile)
                    stop
                elseif((err .eqv. .true.) .and. (trim(errMsg)=='keyword')) then
                    call errorInvalidKeyword(controlDataFile)
                    stop
                elseif((err .eqv. .true.) .and. (trim(errMsg)=='command')) then
                    call errorInvalidCommand(controlDataFile)
                    stop
                elseif((err .eqv. .true.) .and. (trim(errMsg)=='parameter')) then
                    call errorInvalidParameter(controlDataFile)
                    stop
                elseif((err .eqv. .true.) .and. (trim(errMsg)=='datatable')) then
                    call errorCpAtMinCtTable(controlDataFile,errCode)
                    stop
                endif
            !---------------------------------------------
            case ('estimation')
                ! Read the wind speed estimation block
                write(6,*) 'Reading the wind speed estimator block ...'
                call readEstimation(controlDataFile,Cptable,err,errMsg,errCode)
                ! Preparing lambda and pitch list
                allocate(Cptable%LambdaList(Cptable%NumLambdas,1))
                allocate(Cptable%PitchList(Cptable%NumPitchAngles,1))
                Cptable%LambdaList(:,1) = (/(Cptable%Lambda(1)+i*Cptable%Lambda(2),i=0,Cptable%NumLambdas-1)/)
                Cptable%LambdaList(:,1) = FLOAT(INT(Cptable%LambdaList(:,1)*10.0_mk+0.5_mk))/10.0_mk ! rounding to 1 decimal
                
                ! write(6,*) Cptable%LambdaList(:,1)
                Cptable%PitchList(:,1) = (/(Cptable%Pitch(1)+i*Cptable%Pitch(2),i=0,Cptable%NumPitchAngles-1)/)
                Cptable%PitchList(:,1) = FLOAT(INT(Cptable%PitchList(:,1)*10.0_mk+0.5_mk))/10.0_mk ! rounding to 1 decimal
                
               ! write(6,*) Cptable%PitchList(:,1)
                if((err .eqv. .true.) .and. (trim(errMsg)=='block')) then
                    call errorInvalidBlock(controlDataFile)
                    stop
                elseif((err .eqv. .true.) .and. (trim(errMsg)=='keyword')) then
                    call errorInvalidKeyword(controlDataFile)
                    stop
                elseif((err .eqv. .true.) .and. (trim(errMsg)=='command')) then
                    call errorInvalidCommand(controlDataFile)
                    stop
                elseif((err .eqv. .true.) .and. (trim(errMsg)=='parameter')) then
                    call errorInvalidParameter(controlDataFile)
                    stop
                elseif((err .eqv. .true.) .and. (trim(errMsg)=='datatable')) then
                    call errorCpAtMinCtTable(controlDataFile,errCode)
                    stop
                else
                    ! write(6,*) 'line 1:',Cptable%Pitch(1:3)
                    ! write(6,*) 'line 2:',Cptable%Lambda(1:3)
                endif
            !--------------------------------------------
            case ('turbine_property')
                ! Read the wind turbine property block
                ! need to be implemented together with Alan
                write(6,*) ' WARNING: turbine property block is not implemented yet. Ignored!'
                ! Skip the sub-blocks inside this block.
                call skipSection(controlDataFile)
            !--------------------------------------------
            case ('mpc')
                ! Read the mpc block
                write(6,*) ' WARNING: mpc block is not implemented yet. Ignored!'
                ! Skip the sub-blocks inside this block.
                call skipSection(controlDataFile)
            !--------------------------------------------
            case default
                call errorInvalidBlock(controlDataFile)
                err = .true.
                stop
            end select
    !   --------------------------------------------
        case ('exit')
            eof =.true.
            ! Data in additional control parameter file given above is OK
            all_ok=.true.
            if ((all_ok.eqv..true.) .and. (err .eqv. .false.)) then
                write(6,*) ' Additional control commands and parameters are read successfully.'
            else
                write(6,*) ' ERROR: Not all needed Additional control commands and parameters are presented.'
            endif
    !   --------------------------------------------
        case default
            call errorInvalidKeyword(controlDataFile)
            err = .true.
            stop
        end select
    enddo
end subroutine readAdditionalCtrlParameter
!**************************************************************************************************
subroutine readDownRegulation(controlDataFile,outData,err,errMsg,errCode)
    ! Declare the subroutine input arguments.
    type (TcontrolFile),    pointer      :: controlDataFile
    type(TdownRegulationData), intent(inout)  :: outData
    logical,                   intent(inout)  :: err
    character(len=16), intent(inout) :: errMsg
    integer,           intent(inout) :: errCode

    ! Declare the local variables.
    type (Tline) :: line
    type (Tword) :: words
    logical :: jumpOut

    ! Start main part of this subroutine.
    jumpOut=.false.
    do while (jumpOut.eqv..false.)
        call readline(controlDataFile,line)                 ! line is read
        call getwords(line%line,words,controlDataFile)      ! words in line is retrieved
        select case (trim(words%wordArray(1)))
        case ('begin')
            select case (trim(words%wordArray(2)))
            case('table')
                call readTable(controlDataFile,outData,err,errMsg,errCode)
                if (err.eqv..true.) then
                    exit
                else
                    write(6,*) 'MESSAGE: '//'MinCtTable is read successfully from file: '//trim(adjustl(controlDataFile%includedFileName))
                endif
            case default
                ! call errorInvalidBlock(controlDataFile)
                err = .true.
                errMsg = 'block'
                exit
            end select
        case ('')
            continue
        case ('end')
            ! jump out from this block
            jumpOut =.true.
        case default
            ! call errorInvalidKeyword(controlDataFile)
            err = .true.
            errMsg = 'keyword'
            exit
        end select
    enddo
end subroutine  readDownRegulation
!**************************************************************************************************
subroutine readTable(controlDataFile,outTable,err,errMsg,errCode)
    ! Declare the subroutine input argument variables
    type(TcontrolFile),        pointer       :: controlDataFile
    type(TdownRegulationData), intent(inout) :: outTable
    logical ,                  intent(inout) :: err
    character(len=16),         intent(inout) :: errMsg
    integer,                   intent(inout) :: errCode

    ! Declare loacl variables
    type (Tline) :: line
    type (Tword) :: words
    character(len=256) :: filename =''
    real(mk), dimension(MAX_WORDS) :: outParamVec
    integer :: nRow,nCol
    logical :: jumpOut

    ! main part of this subroutine.
    jumpOut=.false.
    do while (jumpOut.eqv..false.)
        ! read lines
        call readline(controlDataFile,line)                 
        ! retrieve words from the line
        call getwords(line%line,words,controlDataFile)                        
        ! Start to check key words
        select case (trim(words%wordArray(1)))
        case ('dim')
            call getParameters(words,2,2,outParamVec,err)
            if(err .eqv. .true.) then
                errMsg = 'parameter'
                exit
            else
                nRow = int(outParamVec(1))
                nCol = int(outParamVec(2))
            endif
        case ('')
            continue
        case ('file')
           filename = trim(words%wordArray(2)) 
           controlDataFile%includedFileName = filename
           call readDataAtMinCt(filename,nRow,nCol,outTable,err,errCode)
           ! errCode = 1: file does not exist 
           !           2: open file error 
           !           3: wrong number of columns
           if((err .eqv. .true.) .and. (errCode == 1)) then
               errMsg = 'datatable'
               exit
           elseif((err .eqv. .true.) .and. (errCode == 2)) then
               errMsg = 'datatable'
               exit
           elseif((err .eqv. .true.) .and. (errCode == 3)) then
               errMsg = 'datatable'
               exit
           endif
        case ('end')
            ! jump out from the begin table ... end table block
            jumpOut =.true.
            ! data table read success 
            err = .false.
        case default
            err = .true.
            errMsg = 'command'
            exit
        end select
    enddo

end subroutine readTable

!**************************************************************************************************
subroutine readDataAtMinCt(dataFile,numRows,numCols,downRegulationData,err,errCode)
    ! Declare subroution input argument variables
    character(64), intent(in) :: dataFile
    integer, intent(in) :: numRows, numCols
    type(TdownRegulationData), intent(inout)::downRegulationData
    logical, intent(inout) :: err
    integer, intent(inout) :: errCode ! 1: file does not exist
                                      ! 2: open file error
                                      ! 3: wrong number of columns

    ! Declare the local variables.
    integer fileUnit
    integer status,i, nCommentLines
    type (Tline) :: line
    downRegulationData%NumLines = numRows
    downRegulationData%NumColumns = numCols

    ! read the pre-defined lookup table for minimum Ct control strategy
    if (fileExists(dataFile)) then
        ! open the lookup table for minimum Ct control strategy
        call getFreeFileUnit(fileUnit)
        open(unit=fileUnit,file=trim(adjustl(dataFile)),iostat=status,action='read')
        nCommentLines = 0;
        if (status==0) then
            ! read one comment line and 2 lines for the table header
            do while(nCommentLines < 3)
                read(unit=fileUnit,fmt='(A)') line
                nCommentLines = nCommentLines + 1
            enddo

            ! read the table data
            do i=1,downRegulationData%NumLines
                read(fileUnit,*,iostat=status) downRegulationData%Pitch(i),downRegulationData%Lambda(i), &
                                               downRegulationData%Ct(i,1), downRegulationData%Cp(i,1), &
                                               downRegulationData%dCp(i)
                if (status /= 0) then
                    err = .true.
                    errCode = 3
                    close(fileUnit)
                    goto 10
                endif
            enddo
        elseif(status /= 0) then
            err = .true.
            errCode = 2
            goto 10
        endif 
        close(fileUnit)
    else
        err = .true.
        errCode = 1
        goto 10
    endif
10  return
end subroutine readDataAtMinCt

!**************************************************************************************************
subroutine readDataCp(dataFile,numRows,numCols,outData,err,errCode)
    ! Declare subroution input argument variables
    character(64), intent(in) :: dataFile
    integer, intent(in) :: numRows, numCols
    type(TCpData), intent(inout):: outData
    logical, intent(inout) :: err
    integer, intent(inout) :: errCode ! 1: file does not exist
                                      ! 2: open file error
                                      ! 3: wrong number of columns

    ! Declare the local variables.
    integer fileUnit
    integer status,i, nCommentLines
    type (Tline) :: line
    outData%NumPitchAngles = numCols ! number of pitch angles
    outData%NumLambdas     = numRows ! number of lambda

    allocate(outData%Cp(numRows,numCols))

    ! read the pre-defined lookup table for minimum Ct control strategy
    if (fileExists(dataFile)) then
        ! open the lookup table for minimum Ct control strategy
        call getFreeFileUnit(fileUnit)
        open(unit=fileUnit,file=trim(adjustl(dataFile)),iostat=status,action='read')
        nCommentLines = 0;
        if (status==0) then
            ! read first 2 lines from the file to get pitch angle and lambda values
            ! This is a fast solution, it should be improved if there is time
            read(fileUnit,*,iostat=status) outData%Pitch(1),outData%Pitch(2),outData%Pitch(3) 
            read(fileUnit,*,iostat=status) outData%Lambda(1),outData%Lambda(2),outData%Lambda(3) 

            do i=1,numRows
                read(fileUnit,*,iostat=status) outData%Cp(i,:)
                if (status /= 0) then
                    err = .true.
                    errCode = 3
                    close(fileUnit)
                    goto 10
                endif
            enddo
        elseif(status /= 0) then
            err = .true.
            errCode = 2
            goto 10
        endif 
        close(fileUnit)
    else
        err = .true.
        errCode = 1
        goto 10
    endif
10  return
end subroutine readDataCp


!**************************************************************************************************
recursive subroutine skipSection(controlDataFile)
    ! Declare the subroutine input arguments.
    type(TcontrolFile),  pointer :: controlDataFile
    ! Declare local variables 
    type (Tline) :: line
    type (Tword) :: words
    logical ::  jumpOut

    ! main part of the subroutine
    jumpOut=.false.
    do while (jumpOut.eqv..false.)
        call readline(controlDataFile,line)        ! line is read
        call getwords(line%line,words,controlDataFile) ! words in line is retrieved
        select case (trim(words%wordArray(1)))
        case ('begin')
            call skipSection(controlDataFile)
        case ('end')
            jumpOut=.true.
        case default
            continue
        end select
    enddo ! while (jumpOut.eqv..false.)
end subroutine skipSection

!**************************************************************************************************
subroutine readEstimation(controlDataFile,outData,err,errMsg,errCode)
! Declare the subroutine input arguments.
    type(TcontrolFile),     pointer   :: controlDataFile
    type(TCpData),     intent(inout)  :: outData
    logical,           intent(inout)  :: err
    character(len=16), intent(inout)  :: errMsg
    integer,           intent(inout)  :: errCode

    ! Declare the local variables.
    type (Tline) :: line
    type (Tword) :: words
    logical :: jumpOut

    ! Start main part of this subroutine.
    jumpOut=.false.
    do while (jumpOut.eqv..false.)
        call readline(controlDataFile,line)                 ! line is read
        call getwords(line%line,words,controlDataFile)      ! words in line is retrieved
        select case (trim(words%wordArray(1)))
        case ('begin')
            select case (trim(words%wordArray(2)))
            case('cptable')
                call readCpTable(controlDataFile,outData,err,errMsg,errCode)
                if (err.eqv..true.) then
                    exit
                else
                    write(6,*) 'MESSAGE: '//trim(words%wordArray(2))//' is read successfully from file:'//trim(adjustl(controlDataFile%includedFileName))
                endif
            case('cttable','pitchangle','lambda')
                ! skip those three sections for now
                call skipSection(controlDataFile)
                write(6,*) ' WARNING: '//trim(words%wordArray(2))//'block is not implemented yet. Ignored!'
            case default
                ! Invalid Block names
                err = .true.
                errMsg = 'block'
                exit
            end select
        case ('')
            continue
        case ('end')
            ! jump out from this block
            jumpOut =.true.
        case default
            ! Invalid keywords
            ! call errorInvalidKeyword(controlDataFile)
            err = .true.
            errMsg = 'keyword'
            exit
        end select
    enddo
end subroutine readEstimation

!**************************************************************************************************
subroutine readCpTable(controlDataFile,outTable,err,errMsg,errCode)
    ! Declare the subroutine input argument variables
    type(TcontrolFile),        pointer       :: controlDataFile
    type(TCpData),             intent(inout) :: outTable
    logical ,                  intent(inout) :: err
    character(len=16),         intent(inout) :: errMsg
    integer,                   intent(inout) :: errCode

    ! Declare loacl variables
    type (Tline) :: line
    type (Tword) :: words
    character(len=256) :: filename =''
    real(mk), dimension(MAX_WORDS) :: outParamVec
    integer :: nRow,nCol
    logical :: jumpOut

    ! main part of this subroutine.
    jumpOut=.false.
    do while (jumpOut.eqv..false.)
        ! read lines
        call readline(controlDataFile,line)                 
        ! retrieve words from the line
        call getwords(line%line,words,controlDataFile)                        
        ! Start to check key words
        select case (trim(words%wordArray(1)))
        case ('dim')
            call getParameters(words,2,2,outParamVec,err)
            if(err .eqv. .true.) then
                errMsg = 'parameter'
                exit
            else
                nRow = int(outParamVec(1))
                nCol = int(outParamVec(2))
            endif
        case ('')
            continue
        case ('file')
           filename = trim(words%wordArray(2)) 
           controlDataFile%includedFileName = filename
           call readDataCp(filename,nRow,nCol,outTable,err,errCode)
           ! errCode = 1: file does not exist 
           !           2: open file error 
           !           3: wrong number of columns
           if((err .eqv. .true.) .and. (errCode == 1)) then
               errMsg = 'datatable'
               exit
           elseif((err .eqv. .true.) .and. (errCode == 2)) then
               errMsg = 'datatable'
               exit
           elseif((err .eqv. .true.) .and. (errCode == 3)) then
               errMsg = 'datatable'
               exit
           endif
        case ('end')
            ! jump out from the begin table ... end table block
            jumpOut =.true.
            ! data table read success 
            err = .false.
        case default
            err = .true.
            errMsg = 'command'
            exit
        end select
    enddo

end subroutine readCpTable

!**************************************************************************************************
function interpolate2d(x,y,x0,y0,x1,y1,f00,f10,f01,f11)              
!   linear 2d interpolation
!   see : http://fourier.eng.hmc.edu/e176/lectures/ch7/node7.html
!   f(x,y) is unknown.
!  f00(x0,y0) --------- f01 (x0,y1)
!   |                          |
!   f(x,y0) ---- f(x,y)----- f(x,y1)
!   |                           |
!   f10(x1,y0)------------f11(x1,y1)

    implicit none
    integer, parameter :: mk = kind(1.0d0)
    real(mk) :: interpolate2d, fxy0, fxy1, D_x, delta_x, D_y, delta_y
    real(mk), intent(in) :: x,y,x0,y0,x1,y1,f00,f10,f01,f11
    
    ! Do linear interpolations in x-direction
    fxy0 = interpolate(x, x0, x1, f00, f10)
    fxy1 = interpolate(x, x0, x1, f01, f11)
    ! Do linear interpolaiton in y-direction
    interpolate2d = interpolate(y,y0,y1,fxy0,fxy1)
	return	  
end function interpolate2d 


function look_up_Cp(Cptable,lambda_q,pitch_q)
    implicit none
    integer, parameter :: mk = kind(1.0d0)
    type(TCpData), intent(in) :: Cptable 
    real(mk), intent(in) :: lambda_q, pitch_q
    real(mk) :: look_up_Cp, lambda0, lambda1, pitch0, pitch1, Cp00, Cp01, Cp10, Cp11
	integer :: i
	real(mk) :: lambda_index, pitch_index
	integer :: lambda_int, pitch_int

	pitch_index = max(min(pitch_q /Cptable%Pitch(2)+(1.0_mk-Cptable%Pitch(1)/Cptable%Pitch(2)),real(Cptable%NumPitchAngles)), 1.0_mk)
	lambda_index = max(min(lambda_q /Cptable%Lambda(2)+(1.0_mk-Cptable%Lambda(1)/Cptable%Lambda(2)),real(Cptable%NumLambdas)), 1.0_mk)
	
	

    lambda0 = real(floor(lambda_index))
    lambda1 = real(ceiling(lambda_index))
    pitch0 = real(floor(pitch_index))
    pitch1 = real(ceiling(pitch_index))
    Cp00 = Cptable%Cp(int(lambda0),int(pitch0))
    Cp01 = Cptable%Cp(int(lambda0),int(pitch1))
    Cp10 = Cptable%Cp(int(lambda1),int(pitch0))
    Cp11 = Cptable%Cp(int(lambda1),int(pitch1))
   
	look_up_Cp = interpolate2d(lambda_index, pitch_index, lambda0, pitch0, lambda1, pitch1, Cp00, Cp10,Cp01, Cp11)
	
	
	return
end function look_up_Cp


function AeroTorqEstimator( GenTorqueRef, GenSpeed, WindEstvar,deltat)
    ! Estimator for aerodynamic torque, which is used to estimate the rotor-effective wind speed.
    ! The users are encouraged to read "Estimation of effective wind speed" by Østergaard et al.
    implicit none 
    integer, parameter :: mk = kind(1.0d0) 
    ! local variable
    real(mk) :: A, C,  err, K, meas , resid, AeroTorqEstimator!<  
    real(mk), dimension(2):: B , u !<  
    ! input
    real(mk), intent(in) :: GenTorqueRef, GenSpeed, deltat
    ! WindEstvar and their temp arrays
    type(TWindEstvar), intent(inout) :: WindEstvar  
    real(mk):: J , delta, est_Qa, P , sum_err, xhat , Q, R, Kp, Ki !<  [Nm]
    ! assign WindEstvar value
    J = WindEstvar%J 
    Q = WindEstvar%Q
    R = WindEstvar%R
    Kp = WindEstvar%Kp
    Ki = WindEstvar%Ki
    P =  WindEstvar%P
    sum_err = WindEstvar%sum_err
    xhat = WindEstvar%xhat
    est_Qa = WindEstvar%est_Qa
    ! Model 
    A = 1.0_mk
    B = (/ deltat/J , - deltat/J  /)
    C = 1.0_mk
    ! Assign input
    u = (/ est_Qa,GenTorqueRef /) 
    ! Kalman filtering + PI to estimate the aerodynamic torque
    xhat = A* xhat + dot_product(B,u)
    P = A*P*(A) + Q
    ! Calculate the Kalman gain 
    K = P*C/(C*P*C + R) ! Kalman gain;
    ! Calculate the measurement residual 
    meas = GenSpeed
    resid = meas - C*xhat
    ! update the state and error covariance estimate 
    xhat = xhat + K*resid
    P = (1-K*C)*P
    ! Estimate the unknown input using PI 
    err = meas - C*xhat
    sum_err = sum_err + err
    est_Qa = Kp * err + Ki *sum_err
    ! Output
    AeroTorqEstimator = est_Qa
    !update WindEstvar for the next iteration
    WindEstvar%P = P
    WindEstvar%sum_err = sum_err
    WindEstvar%xhat = xhat
    WindEstvar%est_Qa = est_Qa
    return
end function AeroTorqEstimator 
!
function GradDesc(estQ, GenSpeed, PitchMean, WindEstvar,Cptable)
! In the current version, gearbox ratio is omitted
    implicit none
    integer, parameter :: mk = kind(1.0d0) 
    real(mk), intent(in) :: estQ, GenSpeed, PitchMean
    ! local variables
    real(mk) :: f, f_deriv, lambda, Cp, GradDesc, dCpdlambda
    ! WindEstvar and their temp arrays
    type(TWindEstvar), intent(inout) :: WindEstvar  
    type(TCpData), intent(in) :: Cptable 
    integer :: i
    ! assign WindEstvar value
    r = WindEstvar%radius
    
    lambda = 8.0_mk ! initial guess
    f = 1.0_mk
    f_deriv =  1.0_mk
   
    
   !write(6,*), omega,pitch
    
    do while (abs(f/f_deriv) > 0.1_mk)
        lambda = lambda - min(max(f/f_deriv,-2.0_mk),2.0_mk)
         Cp = look_up_Cp(Cptable,lambda,PitchMean)
         f = 2.0_mk*estQ/(rho*pi*r**5.0_mk*GenSpeed**2.0_mk) - Cp/lambda**3.0_mk
         dCpdlambda = (look_up_Cp(Cptable,lambda,PitchMean) - look_up_Cp(Cptable,lambda-0.1_mk,PitchMean)) /0.1_mk
         f_deriv =  -(dCpdlambda/lambda**3.0_mk  - 3.0_mk*Cp/lambda**4.0_mk)
         ! Show warning if looping
         i = i +1
         if (i > 10) then
         write(6,*) "Divergence in wind speed estimator!! Lambda = " , lambda
         lambda = lambda + 5.0_mk ! add a random number to break the loop
         i = 0
         endif
      
    end do
    GradDesc = lambda    
    return
end function GradDesc
!**************************************************************************************************
end module dtu_we_controller_fcns
