## ARM

armenv = Environment()

armenv.Append(CFLAGS = "-O0 -g -Wall -fno-common -ffunction-sections -fdata-sections -MD -Wall -DSTM32F4 -DSTM32F427xx -DHSE_VALUE=8000000 -DUSE_USB_FS -mthumb -mcpu=cortex-m4 -mfloat-abi=softfp -mfpu=fpv4-sp-d16")

armenv.Append(LINKFLAGS = "--static -TSTM32F407VG_FLASH.ld -Wl,-Map=miniblink.map -Wl,--gc-sections -mthumb -mcpu=cortex-m4 -mfloat-abi=softfp -mfpu=fpv4-sp-d16 -Wl,--start-group -lc -lgcc -lnosys -Wl,--end-group")

armenv.Append(CPPPATH = [
    "grbl-servo",
    "grbl-sim",
    "grbl",
    "STM32F4xx_HAL_Driver/Inc",
    "CMSIS/Device/ST/STM32F4xx/Include",
    "CMSIS/Include",
    ])

armenv['CC'] = "/Users/rob/Projecten/stm/gcc-arm-none-eabi-4_8-2014q2/bin/arm-none-eabi-gcc"


armsources = [
    "CMSIS/Device/ST/STM32F4xx/Source/Templates/system_stm32f4xx.c",
    "CMSIS/Device/ST/STM32F4xx/Source/Templates/gcc/startup_stm32f407xx.S",
    "grbl-servo/main.c",
    "grbl-servo/os.c",
    "grbl-servo/serial.c",
    "grbl-servo/interface.c",
    "grbl-servo/servo.c",
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


#objects = [env.Object(source = source) for source in sources]

grbl = armenv.Program(target = 'grbl.elf', source = commonsources + armsources)


#program = Command( None, 'grbl.elf', '/opt/local/bin/arm-none-eabi-gdb -ex "target remote :4242" -ex "load grbl.elf" -ex "set{int}0xE000ED0C=0x5fa0004" -ex "disconnect" -ex "quit"')

#AlwaysBuild(program)
