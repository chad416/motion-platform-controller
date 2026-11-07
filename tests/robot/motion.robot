*** Settings ***
Documentation       motion_fw integration tests (Renode). IDs refer to SPEC.md §9.2.
...                 The firmware runs on the emulated STM32H753 with its simulated actuators;
...                 sim/host_feed.py streams the vehicle over CAN on virtual time and records
...                 every frame the controller sends, which motionlib.py then checks.
Resource            common.resource
Library             Collections
Test Setup          Reset Emulation
Test Teardown       Test Teardown

*** Test Cases ***
IT01 Boot Is Off Then Homes To Park
    [Documentation]    F08, F14: boots into OFF with the servos off and the legs on their end
    ...                stops; ENGAGE is refused; ENABLE -> HOMING -> PARKED (legs at the lower
    ...                soft limit, homed at <= 0.05 m/s).
    Boot
    Advance                     0.2
    Host                        mh.cmd(${OP_ENGAGE})
    Advance                     0.1
    Host                        mh.cmd(${OP_ENABLE})
    Wait For Line On Uart       STATE HOMING
    Wait For Line On Uart       STATE PARKED    timeout=2
    Advance                     0.3
    Shell                       motion version
    Wait For Line On Uart       motion_fw 0.3.0
    # F14 diagnostics: every shell report answers
    Shell                       motion status
    Wait For Line On Uart       content=state=PARKED phase=
    Shell                       motion legs
    Wait For Line On Uart       content=leg6 length=
    Shell                       motion pose
    Wait For Line On Uart       content=cmd x=
    Shell                       motion params
    Wait For Line On Uart       content=force_gain=0.500
    Shell                       motion plant
    Wait For Line On Uart       content=leg6 true_length=
    Shell                       motion timing
    Wait For Line On Uart       content=TIMING cue cycles=
    ${p}=                       Dump Frames    it01
    ${acks}=                    Acks    ${p}
    Should Be Equal             ${acks}[0]    2 0 ${ST_BAD_STATE} 0    ENGAGE must be refused in OFF
    Should Be Equal             ${acks}[1]    1 0 ${ST_OK} 0
    ${st}=                      State At    ${p}    150
    Should Be Equal             ${st}    OFF
    ${s0}=                      Strokes After    ${p}    0
    FOR    ${v}    IN    @{s0}
        Should Be True          ${v} < 0.02    leg not on its end stop at boot: ${v} mm
    END
    ${t_end}=                   Virtual Time Ms
    ${s1}=                      Strokes After    ${p}    ${{ $t_end - 100 }}
    FOR    ${v}    IN    @{s1}
        Should Be Near          ${v}    ${SOFT_LO_MM}    0.02    parked stroke (mm)
    END
    ${v}=                       Max Leg Speed    ${p}    0    ${t_end}
    Should Be True              ${v} <= 0.0505    homing speed ${v} m/s
    Log                         homing max leg speed ${v} m/s

IT02 Engage To Neutral And Park
    [Documentation]    F08, F02: ENGAGE -> ENGAGED within 3.5 s; the FK pose is neutral within
    ...                0.5 mm and 0.05 deg; PARK -> PARKED.
    Boot
    Enable And Park
    Host                        mh.start()
    Host                        mh.cmd(${OP_ENGAGE})
    ${a}=                       Wait For Line On Uart    STATE ENGAGING
    ${b}=                       Wait For Line On Uart    STATE ENGAGED    timeout=4
    ${ta}=                      Log Time Ms    ${a}[Line]
    ${tb}=                      Log Time Ms    ${b}[Line]
    Should Be True              ${tb} - ${ta} <= 3500    engage took ${{ $tb - $ta }} ms
    Advance                     0.5
    ${t}=                       Virtual Time Ms
    Advance                     0.1
    ${p}=                       Dump Frames    it02
    ${q}=                       Pose After    ${p}    ${t}
    Log                         engaged pose ${q}
    FOR    ${k}    IN RANGE    3
        Should Be True          abs(${q}[${k}]) <= 0.5    position ${k}: ${q}[${k}] mm
        Should Be True          abs(${q}[${{ $k + 3 }}]) <= 0.05    angle ${k}: ${q}[${{ $k + 3 }}] deg
    END
    ${st}=                      Last Status    ${p}
    Should Be True              ${st}[fk_ok]
    Host                        mh.cmd(${OP_PARK})
    Wait For Line On Uart       STATE PARKING
    Wait For Line On Uart       STATE PARKED    timeout=4

IT03 Surge Step Washes Out With Tilt
    [Documentation]    F04, P04, P05 on the delivered motion (FK pose over CAN): a 3 m/s^2
    ...                surge step for 8 s (no body rotation, so pitch is the tilt alone) surges the platform, the translation returns to neutral
    ...                within 6 s, pitch holds the -8.8 deg tilt cue, and the tilt rate stays
    ...                at 3 deg/s (+0.1 deg/s: POSE_R resolution over 100 ms).
    Boot
    Engage
    ${t0}=                      Play    surge_step
    Advance                     16.5
    ${p}=                       Dump Frames    it03
    ${ext}=                     Pose Extremes    ${p}    ${t0}    ${{ $t0 + 3000 }}
    Log                         launch onset peaks ${ext}
    Should Be True              ${ext}[0] > 50    surge onset only ${ext}[0] mm
    # the step starts at 1 s: back within 5 mm by 7 s (P05)
    ${late}=                    Pose Extremes    ${p}    ${{ $t0 + 7000 }}    ${{ $t0 + 9000 }}
    Should Be True              ${late}[0] < 5    still ${late}[0] mm from neutral 6 s after the step
    ${q}=                       Pose After    ${p}    ${{ $t0 + 8900 }}
    Should Be Near              ${q}[4]    -8.80    0.1    pitch (deg) holding the 1.5 m/s^2 cue
    ${rate}=                    Max Tilt Rate    ${p}    ${t0}    ${{ $t0 + 16000 }}
    Log                         max tilt rate ${rate} deg/s
    Should Be True              ${rate} <= 3.1    tilt rate ${rate} deg/s

IT04 Lane Change Tracking
    [Documentation]    F04, P02: lane change scenario; leg tracking error (setpoint - encoder,
    ...                at 1 kHz in the servo thread) <= 1 mm RMS and <= 3 mm peak.
    Boot
    Engage
    Shell                       motion track reset
    Wait For Line On Uart       tracking reset
    ${t0}=                      Play    lane_change
    Advance                     10.5
    Shell                       motion track
    ${l}=                       Wait For Line On Uart    TRACK samples=
    ${m}=                       Evaluate    re.search(r"samples=(\\d+) rms_um=([\\d.]+) peak_um=([\\d.]+)", $l["Line"])    modules=re
    Log                         ${l}[Line]
    Should Be True              int($m.group(1)) > 9000    too few samples
    Should Be True              float($m.group(2)) <= 1000    RMS ${m.group(2)} um
    Should Be True              float($m.group(3)) <= 3000    peak ${m.group(3)} um

IT05 Excessive Cue Saturates But Stays In Limits
    [Documentation]    F03, P03: a 3 g cue sets SATURATED; every leg stroke in every LEG_POS
    ...                frame stays inside the soft limits (10..240 mm). The setpoints keep 0.1 mm
    ...                inside them, so the servo tracking error cannot carry a leg past.
    Boot
    Engage
    ${t0}=                      Play    excessive
    Advance                     8.5
    ${p}=                       Dump Frames    it05
    ${sat}=                     Status Flag Seen    ${p}    saturated    ${t0}
    Should Be True              ${sat}    SATURATED never reported
    ${scale}=                   Min Ws Scale    ${p}
    Log                         minimum workspace scale ${scale}
    ${r}=                       Stroke Range    ${p}    ${t0}
    Log                         stroke range ${r} mm
    Should Be True              ${r}[0] >= ${SOFT_LO_MM}    leg below the soft limit: ${r}[0] mm
    Should Be True              ${r}[1] <= ${SOFT_HI_MM}    leg above the soft limit: ${r}[1] mm
    ${st}=                      Last Status    ${p}
    Should Be Equal             ${st}[state]    ENGAGED    excessive cue must not fault

IT06 Estop Stops Then Descends
    [Documentation]    F11, P06: SW1 (the normally-closed E-stop circuit opening) while the
    ...                platform moves -> ESTOP, standstill <= 0.5 s,
    ...                then a descent at <= 0.05 m/s; RESET refused until parked, then PARKED.
    Boot
    Engage
    ${t0}=                      Play    launch
    Advance                     1.4
    ${tp}=                      Virtual Time Ms
    Execute Command             sysbus.gpioPortG.sw1 Press
    Wait For Line On Uart       STATE ESTOP
    Host                        mh.cmd(${OP_RESET})
    Advance                     0.1
    Execute Command             sysbus.gpioPortG.sw1 Release
    Advance                     0.1
    Host                        mh.cmd(${OP_RESET})
    Advance                     6.5
    Shell                       motion fault
    Wait For Line On Uart       content=descended=1
    Host                        mh.cmd(${OP_RESET})
    Wait For Line On Uart       STATE PARKED
    Advance                     0.1
    ${t_end}=                   Virtual Time Ms
    ${p}=                       Dump Frames    it06
    ${v0}=                      Max Leg Speed    ${p}    ${{ $tp - 300 }}    ${tp}
    Should Be True              ${v0} > 0.02    legs were not moving at the E-stop (${v0} m/s)
    ${ts}=                      Standstill Time    ${p}    ${tp}
    Log                         E-stop at ${v0} m/s: standstill after ${{ $ts - $tp }} ms
    Should Be True              0 < ${ts} - ${tp} <= 500    standstill after ${{ $ts - $tp }} ms
    ${vd}=                      Max Leg Speed    ${p}    ${ts}    ${{ $ts + 7000 }}
    Log                         descent max leg speed ${vd} m/s
    Should Be True              ${vd} <= 0.0505    descent ${vd} m/s
    ${f}=                       Fault Frames    ${p}
    ${code}=                    Evaluate    $f[0].split()[1:4]
    Should Be Equal             ${code}    ${{["6", "0", "7"]}}    FAULT frame: ESTOP, no leg, state ESTOP
    ${acks}=                    Acks    ${p}
    Should Be Equal             ${acks}[-3]    4 0 ${ST_ESTOP} 0    RESET refused while the E-stop is open
    Should Be Equal             ${acks}[-2]    4 0 ${ST_BAD_STATE} 0    RESET must wait for the descent
    Should Be Equal             ${acks}[-1]    4 0 ${ST_OK} 0
    ${r}=                       Strokes After    ${p}    ${{ $t_end - 100 }}
    FOR    ${v}    IN    @{r}
        Should Be Near          ${v}    ${SOFT_LO_MM}    0.02    stroke after the descent (mm)
    END

IT07 Heartbeat Loss Washes Out Then Parks
    [Documentation]    F12: VEHICLE_STATE stops for > 200 ms while ENGAGED -> COMMS_LOST event
    ...                (FAULT frame), washout to neutral, then PARKING -> PARKED; not a latched fault.
    Boot
    Engage
    ${t0}=                      Play    launch
    Advance                     4
    ${ts}=                      Virtual Time Ms
    Host                        mh.stop()
    Wait For Line On Uart       EVENT COMMS_LOST
    Wait For Line On Uart       STATE PARKING    timeout=12
    Wait For Line On Uart       STATE PARKED    timeout=5
    Advance                     0.1
    ${p}=                       Dump Frames    it07
    ${f}=                       Fault Frames    ${p}
    ${t_lost}=                  Evaluate    float($f[0].split()[0])
    ${code}=                    Evaluate    $f[0].split()[1]
    Should Be Equal             ${code}    7    FAULT frame COMMS_LOST
    Log                         comms loss reported ${{ $t_lost - $ts }} ms after the last sample
    Should Be True              200 < ${t_lost} - ${ts} <= 230
    ${seen}=                    Status Flag Seen    ${p}    comms_lost    ${ts}
    Should Be True              ${seen}
    ${st}=                      Last Status    ${p}
    Should Be Equal As Integers    ${st}[fault]    0    comms loss must not latch a fault
    ${tpark}=                   First State Time    ${p}    PARKING    ${ts}
    ${q}=                       Pose After    ${p}    ${tpark}
    Log                         pose when parking starts ${q}
    Should Be True              abs(${q}[4]) < 0.6    pitch not washed out: ${q}[4] deg

IT08 Seized Leg Faults And Descends
    [Documentation]    F13: leg 3 seizes half-way through ENGAGING, when every leg moves at
    ...                72 mm/s: the error passes 5 mm after ~70 ms, so the following-error FAULT
    ...                on leg 3 must come within 70 + 50 ms (+ one cueing step and the log
    ...                latency: <= 150 ms). The other legs stop and descend at <= 0.05 m/s;
    ...                once freed, RESET re-homes through HOMING to PARKED.
    Boot
    Enable And Park
    Host                        mh.start()
    Host                        mh.cmd(${OP_ENGAGE})
    Wait For Line On Uart       STATE ENGAGING
    Advance                     1.5
    Shell                       motion seize 3 1
    ${a}=                       Wait For Line On Uart    SEIZE leg=3 on
    ${b}=                       Wait For Line On Uart    FAULT FOLLOWING leg=3
    ${ta}=                      Log Time Ms    ${a}[Line]
    ${tb}=                      Log Time Ms    ${b}[Line]
    Log                         following-error fault ${{ $tb - $ta }} ms after the leg seized
    Should Be True              ${tb} - ${ta} <= 150
    Wait For Line On Uart       STATE FAULT
    ${tf}=                      Virtual Time Ms
    Advance                     7
    Shell                       motion fault
    Wait For Line On Uart       content=descended=1
    Shell                       motion seize 3 0
    Wait For Line On Uart       SEIZE leg=3 off
    Host                        mh.cmd(${OP_RESET})
    Wait For Line On Uart       STATE HOMING
    Wait For Line On Uart       STATE PARKED    timeout=8
    Advance                     0.1
    ${p}=                       Dump Frames    it08
    ${vd}=                      Max Leg Speed    ${p}    ${tf}    ${{ $tf + 7000 }}    12456
    Log                         descent max speed (legs 1, 2, 4, 5, 6) ${vd} m/s
    Should Be True              ${vd} <= 0.0505
    ${f}=                       Fault Frames    ${p}
    ${code}=                    Evaluate    $f[0].split()[1:3]
    Should Be Equal             ${code}    ${{["1", "3"]}}    FAULT frame: FOLLOWING on leg 3
    ${acks}=                    Acks    ${p}
    Should Be Equal             ${acks}[-1]    4 0 ${ST_OK} 0

IT09 Ten Minutes Of Cueing Without Overruns
    [Tags]    long
    [Documentation]    P01: 600 s of virtual time ENGAGED, playing all seven scenarios in turn
    ...                (including the 3 g cue, where the workspace bisection runs every step):
    ...                zero servo and cueing overruns, worst cycle times inside their periods,
    ...                no fault, no lost vehicle sample, no lost STATUS frame. About 16 min wall.
    Boot
    Engage
    ${names}=                   Create List    launch    braking    lane_change    cornering
    ...                         bump    excessive    surge_step
    ${files}=                   Evaluate    "[" + ",".join("'${SIM_DIR}/data/%s.csv'" % n for n in $names) + "]"
    ${t0}=                      Virtual Time Ms
    Host                        mh.playlist(${files}, 600)
    FOR    ${i}    IN RANGE    60
        Advance                 10
    END
    Advance                     1.1
    Shell                       motion timing
    ${ls}=                      Wait For Line On Uart    TIMING servo cycles=
    ${lc}=                      Wait For Line On Uart    TIMING cue cycles=
    ${lv}=                      Wait For Line On Uart    vehicle samples=
    Log                         ${ls}[Line]
    Log                         ${lc}[Line]
    Log                         ${lv}[Line]
    ${t_end}=                   Virtual Time Ms
    ${p}=                       Dump Frames    it09
    ${tm}=                      Timing Frames    ${p}
    Log                         last TIMING frame (servo us, cue us, overruns) ${tm}[-1]
    Should Be True              len($tm) >= 600    only ${{ len($tm) }} TIMING frames
    Should Be True              ${tm}[-1][2] == 0 and ${tm}[-1][3] == 0    overruns ${tm}[-1]
    Should Be True              ${tm}[-1][0] < 1000    servo worst ${tm}[-1][0] us
    Should Be True              ${tm}[-1][1] < 2000    cueing worst ${tm}[-1][1] us
    ${f}=                       Fault Frames    ${p}
    Should Be Empty             ${f}    no fault expected
    ${st}=                      Last Status    ${p}
    Should Be Equal             ${st}[state]    ENGAGED
    ${g}=                       Seq Gaps    ${p}    0x181
    Should Be Equal As Integers    ${g}    0    STATUS frames lost
    ${n}=                       Frame Count    ${p}    0x181
    Log                         ${n} STATUS frames over ${{ ($t_end - $t0) / 1000 }} s virtual
    Should Contain              ${lv}[Line]    dropped=0
    ${m}=                       Evaluate    int(re.search(r"samples=(\\d+)", $lv["Line"]).group(1))    modules=re
    Should Be True              ${m} >= 60000    only ${m} vehicle samples

IT10 Washout Parameter Over CAN
    [Documentation]    F09: GET_PARAM / SET_PARAM over CAN. The force gain is read (0.500),
    ...                set to 0.250 and read back; out-of-range values and unknown keys are
    ...                rejected and change nothing; a SET while ENGAGED is refused (the filters
    ...                are re-initialised). The new gain is applied: under the 3 m/s^2 surge step
    ...                the pitch holds asin(0.25 * 3 / g) = -4.39 deg instead of -8.80 deg, and the
    ...                onset peak (linear in the gain) halves from IT03's 81.9 mm.
    Boot
    Enable And Park
    Host                        mh.cmd(${OP_GET}, ${KEY_FORCE_GAIN})
    Host                        mh.cmd(${OP_SET}, ${KEY_FORCE_GAIN}, 250)
    Host                        mh.cmd(${OP_GET}, ${KEY_FORCE_GAIN})
    Host                        mh.cmd(${OP_SET}, ${KEY_FORCE_GAIN}, 1500)
    Host                        mh.cmd(${OP_SET}, 99, 1)
    Host                        mh.cmd(${OP_GET}, ${KEY_FORCE_GAIN})
    Advance                     0.1
    Host                        mh.start()
    Host                        mh.cmd(${OP_ENGAGE})
    Wait For Line On Uart       STATE ENGAGED    timeout=4
    Host                        mh.cmd(${OP_SET}, ${KEY_FORCE_GAIN}, 300)
    ${t0}=                      Play    surge_step
    Advance                     9.5
    ${p}=                       Dump Frames    it10
    ${acks}=                    Acks    ${p}
    Log                         ${acks}
    Should Be Equal             ${acks}[0]    1 0 ${ST_OK} 0    ENABLE
    Should Be Equal             ${acks}[1]    6 1 ${ST_OK} 500    default force gain
    Should Be Equal             ${acks}[2]    5 1 ${ST_OK} 250
    Should Be Equal             ${acks}[3]    6 1 ${ST_OK} 250    read back
    Should Be Equal             ${acks}[4]    5 1 ${ST_BAD_VALUE} 1500    gain above 1 must be rejected
    Should Be Equal             ${acks}[5]    5 99 ${ST_BAD_KEY} 1
    Should Be Equal             ${acks}[6]    6 1 ${ST_OK} 250    rejected SETs must not change it
    Should Be Equal             ${acks}[8]    5 1 ${ST_BAD_STATE} 300    SET while ENGAGED
    ${ext}=                     Pose Extremes    ${p}    ${t0}    ${{ $t0 + 3000 }}
    Log                         surge onset peak ${ext}[0] mm with gain 0.25
    Should Be Near              ${ext}[0]    40.95    2    onset peak (mm), half of 81.9
    ${q}=                       Pose After    ${p}    ${{ $t0 + 8900 }}
    Log                         pitch ${q}[4] deg with gain 0.25
    Should Be Near              ${q}[4]    -4.39    0.1    pitch (deg) holding the 0.75 m/s^2 cue
