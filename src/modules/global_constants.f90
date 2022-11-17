module global_constants
    
    ! Constants
    integer, parameter :: mk = kind(1.0d0)
    real(mk) pi, degrad, raddeg, rho
    parameter(pi = 3.14159265358979_mk, degrad = 0.01745329251994_mk, raddeg = 57.295779513093144_mk, rho = 1.225_mk)
    ! More constant
    integer MAXWPLINES, WORD_LENGTH, MAX_WORDS
    integer MAX_NUM_Lambda, MAX_NUM_Pitch
    parameter(MAXWPLINES=100)
    parameter(MAX_NUM_Lambda=100)
    parameter(MAX_NUM_Pitch=100)
    parameter(WORD_LENGTH=256)
    parameter(MAX_WORDS=50)
    logical :: newtimestep=.TRUE.
    
end module global_constants