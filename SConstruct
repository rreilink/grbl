## ARM

armenv = Environment(
    CC = "/Users/rob/Projecten/stm/gcc-arm-none-eabi-5_4-2016q3/bin/arm-none-eabi-gcc",
    CCFLAGS = 
        "-g -c -Wall -Wextra -Wno-unused-parameter -Wno-missing-field-initializers -fmessage-length=0 "
        "-fno-exceptions -fno-builtin -ffunction-sections -fdata-sections -funsigned-char -MMD "
        "-fno-delete-null-pointer-checks -fomit-frame-pointer "
        "-mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=softfp "
        "-O0 -std=gnu99 -DSTM32F401xE -D__FPU_PRESENT=1 -DARM_MATH_CM4 -MMD -MP "
        "-DHSE_VALUE=8000000"
    ,
    CPPPATH = [
        "grbl-servo",
        "grbl-sim",
        "grbl",
        "STM32F4xx_HAL_Driver/Inc",
        "CMSIS/Device/ST/STM32F4xx/Include",
        "CMSIS/Include",    
        ]
    ,
    LINKFLAGS =
        "-Wl,--gc-sections "
        "-mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=softfp "
        "-TSTM32F401XE.ld "    
    ,
    LINKCOM = "$LINK $LINKFLAGS $__RPATH -o $TARGET $SOURCES $_LIBDIRFLAGS $_LIBFLAGS"
    )



armsources = [
    "CMSIS/Device/ST/STM32F4xx/Source/Templates/system_stm32f4xx.c",
    "CMSIS/Device/ST/STM32F4xx/Source/Templates/gcc/startup_stm32f401xe.S",
    "grbl-servo/main.c",
    "grbl-servo/os.c",
    "grbl-servo/serial.c",
    "grbl-servo/interface.c",
    "grbl-servo/servo.c",
    "grbl-servo/tempcontrol.c",
    "grbl/probe.c",
    ] + Glob('STM32F4xx_HAL_Driver/Src/*.c')



## Common

commonsources = [
    'grbl/nuts_bolts.c',
    'grbl/planner.c',
    'grbl/gcode.c',
    'grbl/motion_control.c',
    
    'grbl/protocol.c',
    'grbl/report.c',
    'grbl/print.c',
    'grbl/settings.c',
    
    'grbl-sim/grbl_eeprom_extensions.c',
    
    'grbl-servo/system.c',
    'grbl-servo/pathplanner.c',
    ]

##
simsources = [
    
    'grbl-sim/avr/eeprom.c',

    'sim.c',

]



simenv = Environment()
simenv.Append(CPPPATH = ['grbl-sim'], CPPDEFINES = {'_USE_MATH_DEFINES' : '', 'NULL': '((void *)0)'})

#simenv.SharedLibrary(target = 'grbl.dll', source = commonsources + simsources)

grbl = armenv.Program(target = 'grbl.elf', source = commonsources + armsources, LIBS = ['m' ,'c', 'gcc', 'nosys'])


#program = Command( None, 'grbl.elf', '/opt/local/bin/arm-none-eabi-gdb -ex "target remote :4242" -ex "load grbl.elf" -ex "set{int}0xE000ED0C=0x5fa0004" -ex "disconnect" -ex "quit"')

#AlwaysBuild(program)
