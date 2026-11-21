$ErrorActionPreference = 'Stop'
Push-Location (Split-Path $PSScriptRoot -Parent)
try {
    $toolBin = 'D:\keil5\ARM\Version5.06\bin'
    $includes = @('-I.', '-I..\CORE', '-I..\FWLIB\inc')
    $includes += Get-ChildItem ..\SYSTEM -Directory | ForEach-Object { '-I' + $_.FullName }
    $flags = @('-c', '--cpu', 'Cortex-M4', '--fpu=SoftVFP', '--c99', '--split_sections', '-O1', '-DSTM32F40_41xxx', '-DUSE_STDPERIPH_DRIVER')
    & "$toolBin\armcc.exe" @flags @includes tests\serial_tune_test.c -o build\serial_tune_test.o
    if ($LASTEXITCODE) { throw 'Test compile failed' }
    & "$toolBin\armcc.exe" @flags @includes ..\FWLIB\src\stm32f4xx_usart.c -o build\serial_tune_usart.o
    if ($LASTEXITCODE) { throw 'USART compile failed' }
    # Unused peripheral entry points trap if a test unexpectedly calls one.
    $trapNames = @('Count_Distance','EXTIXa_Init','GPIO_Init','GPIO_PinAFConfig',
        'KUAIDIYITI2','Key_Init','LED_Init','Left_Encoder_Init','Motor_Init',
        'Motor_PWM_Init','NVIC_Init','OLED_Init','OLED_Refresh','OLED_ShowString',
        'RCC_AHB1PeriphClockCmd','RCC_APB1PeriphClockCmd','Right_Encoder_Init',
        'TIM_ClearITPendingBit','TIM_GetCounter','TIM_GetITStatus','TIM_SetCounter',
        'TIMer2_Init','TIMer4_Init','TIMer6_Init','board_init','delay_ms',
        'uart1_init','uart2_init','uart3_init','RCC_APB1PeriphResetCmd',
        'RCC_APB2PeriphResetCmd','RCC_GetClocksFreq')
    $trapCode = ($trapNames | ForEach-Object { "void $_(void) { for (;;) {} }" }) -join "`n"
    Set-Content -LiteralPath build\serial_tune_traps.c -Value $trapCode -Encoding ASCII
    & "$toolBin\armcc.exe" @flags build\serial_tune_traps.c -o build\serial_tune_traps.o
    if ($LASTEXITCODE) { throw 'Trap compile failed' }
    & "$toolBin\armlink.exe" build\serial_tune_test.o build\serial_tune_usart.o build\serial_tune_traps.o --entry=test_main --ro-base=0x08000000 --rw-base=0x20000000 --datacompressor=off -o build\serial_tune_test.axf
    if ($LASTEXITCODE) { throw 'Test link failed' }
    $env:PYTHONPATH = (Join-Path (Get-Location) 'build\test_deps')
    & "$env:USERPROFILE\.cache\codex-runtimes\codex-primary-runtime\dependencies\python\python.exe" tests\run_serial_tune_test.py build\serial_tune_test.axf
    if ($LASTEXITCODE) { throw 'Emulator test failed' }
} finally { Pop-Location }
