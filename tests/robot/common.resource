*** Settings ***
Documentation       Shared setup for motion_fw integration tests in Renode.
Resource            ${RENODEKEYWORDS}
Library             ${CURDIR}/motionlib.py

*** Variables ***
# Override from the command line: --variable ELF:/path/to/zephyr.elf
${ELF}              /root/trainer-ws/build/motion/zephyr/zephyr.elf
${SIM_DIR}          ${CURDIR}/../../sim
${UART}             sysbus.usart2

# CMD opcodes and status codes (protocol/motion.dbc, mcue/ctl.h)
${OP_ENABLE}        1
${OP_ENGAGE}        2
${OP_PARK}          3
${OP_RESET}         4
${OP_SET}           5
${OP_GET}           6
${ST_OK}            0
${ST_BAD_STATE}     1
${ST_ESTOP}         2
${ST_BAD_KEY}       3
${ST_BAD_VALUE}     4
# SET_PARAM / GET_PARAM keys (mcue/ctl.h mc_param_key)
${KEY_FORCE_GAIN}   1

# Leg limits as stroke above the lower end stop (SPEC §3): soft 10 / 240 mm
${SOFT_LO_MM}       10.0
${SOFT_HI_MM}       240.0

*** Keywords ***
Create Platform
    Execute Command             $elf=@${ELF}
    Execute Command             include @${SIM_DIR}/motion.resc
    # deterministic host stand-in on the CAN bus (virtual-time 100 Hz stream + frame log)
    Execute Command             include @${SIM_DIR}/host_feed.py
    Execute Command             python "mh = MotionHost(self.Machine)"
    Create Terminal Tester      ${UART}    defaultPauseEmulation=True

Boot
    Create Platform
    Start Emulation
    Wait For Line On Uart       READY state=OFF

Advance
    [Documentation]    Run the emulation for a fixed virtual time.
    [Arguments]    ${seconds}
    Execute Command             pause
    Execute Command             emulation RunFor "${seconds}"

Host
    [Documentation]    Call the host stand-in (IronPython inside Renode).
    [Arguments]    ${code}
    ${out}=    Execute Command    python "${code}"
    RETURN    ${out}

Shell
    [Arguments]    ${command}
    Write Line To Uart          ${command}

Enable And Park
    Host                        mh.cmd(${OP_ENABLE})
    Wait For Line On Uart       STATE HOMING
    Wait For Line On Uart       STATE PARKED    timeout=2

Engage
    [Documentation]    ENABLE, start streaming the vehicle at rest, ENGAGE, wait for ENGAGED.
    Enable And Park
    Host                        mh.start()
    Host                        mh.cmd(${OP_ENGAGE})
    Wait For Line On Uart       STATE ENGAGING
    Wait For Line On Uart       STATE ENGAGED    timeout=4

Play
    [Documentation]    Stream a scenario (sim/data/<name>.csv) from now; returns its start (ms).
    [Arguments]    ${name}
    ${t0}=    Virtual Time Ms
    Host                        mh.load('${SIM_DIR}/data/${name}.csv')
    RETURN    ${t0}

Dump Frames
    [Documentation]    Write the recorded controller frames; returns the CSV path.
    [Arguments]    ${name}
    ${path}=    Set Variable    ${OUTPUT DIR}/${name}_can.csv
    Host                        mh.dump('${path}')
    RETURN    ${path}

Virtual Time Ms
    ${s}=     Execute Command    machine ElapsedVirtualTime
    ${m}=     Evaluate    re.search(r"(\\d+):(\\d+):(\\d+)\\.(\\d+)", $s)    modules=re
    Should Not Be Equal    ${m}    ${None}    cannot parse virtual time: ${s}
    ${ms}=    Evaluate    ((int($m.group(1)) * 60 + int($m.group(2))) * 60 + int($m.group(3))) * 1000 + int(($m.group(4) + "000000")[:6]) / 1000
    RETURN    ${ms}

Log Time Ms
    [Documentation]    Firmware time of a Zephyr log line "[hh:mm:ss.mmm,uuu] ..." in ms.
    [Arguments]    ${line}
    ${m}=    Evaluate    re.search(r"\\[(\\d+):(\\d+):(\\d+)\\.(\\d+),(\\d+)\\]", $line)    modules=re
    Should Not Be Equal    ${m}    ${None}    no log timestamp in: ${line}
    ${ms}=   Evaluate    ((int($m.group(1)) * 60 + int($m.group(2))) * 60 + int($m.group(3))) * 1000 + int($m.group(4)) + int($m.group(5)) / 1000
    RETURN    ${ms}

Should Be Near
    [Arguments]    ${value}    ${expected}    ${tol}    ${what}
    Should Be True    abs(${value} - (${expected})) <= ${tol}    ${what}: ${value}, expected ${expected} +- ${tol}
