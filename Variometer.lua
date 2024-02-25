--[[
    Author: flightwusel
    Helpful: https://xp-soaring.github.io/instruments/index.html
]]

require("graphics")

local settings = {
    smoothing = 1., -- x seconds attack
}

local integrateStep_s = 1.
local integrateDuration_s = 30.

-- workaround for LR total energy bug
local lr_totalEnergy_compensationFactor = 2.

--[[ runtime variables ]]

local integrateBuffer = {}
local integrateBufferSum = 0.
local integrateBufferEarliest = 0.
local integrateBufferIndex = 1
local integrateBufferIndexTime_s -- initialized during activate()
local integrateBufferSteps = math.ceil(integrateDuration_s / integrateStep_s)
for _i = 1, integrateBufferSteps do
    integrateBuffer[_i] = 0.
end

local integrated_mPerS = 0.
local vy_mPerS = 0.
local vyOwn_mPerS = 0.
local vySmoothed_mPerS = 0.
local totalEnergy_mPerS = 0.
local totalEnergySmoothed_mPerS = 0.
local totalEnergyOwnCalc_mPerS = 0.
local totalEnergyOwnCalcSmoothed_mPerS = 0.

-- ok to be nil
local iasLast_mPerS
local yLast_m

local y_dataref = XPLMFindDataRef("sim/flightmodel/position/local_y")
local pressureAlt_dataref = XPLMFindDataRef("sim/flightmodel2/position/pressure_altitude")
local vy_dataref = XPLMFindDataRef("sim/flightmodel/position/local_vy")
local ias_kt_dataref = XPLMFindDataRef("sim/flightmodel/position/indicated_airspeed2")
local tas_mPerS_dataref = XPLMFindDataRef("sim/flightmodel/position/true_airspeed")
local period_s_dataref = XPLMFindDataRef("sim/operation/misc/frame_rate_period")
local totalEnergy_ftPerMin_dataref = XPLMFindDataRef("sim/cockpit2/gauges/indicators/total_energy_fpm")
local time_s_dataref = XPLMFindDataRef("sim/time/total_running_time_sec")


--[[ local ]]

local function log(msg, level)
    -- @see https://stackoverflow.com/questions/9168058/how-to-dump-a-table-to-console
    local function dump(o)
        if type(o) == 'table' then
            local s = '{ '
            for k,v in pairs(o) do
                if type(k) ~= 'number' then k = '"'..k..'"' end
                s = s .. '['..k..'] = ' .. dump(v) .. ','
            end
            return s .. '} '
        else
            return tostring(o)
        end
    end

    local msg = msg or ""
    local level = level or ""
    local filePath = debug.getinfo(2, "S").source
    local fileName = filePath:match("[^/\\]*.lua$")
    local functionName = debug.getinfo(2, "n").name
    logMsg(
        string.format(
            "%s::%s() %s%s",
            fileName,
            functionName,
            level,
            dump(msg)
        )
    )
end

local function err(msg)
    return log(msg, "[ERROR] ")
end

local function addToggleMacroAndCommand(cmdRef, titlePrefix, activateCallbackName, globalStateVariableName)
    local macroActivated = loadstring("return " .. globalStateVariableName)() and 'activate' or 'deactivate'
    local cmdRefToggle = cmdRef .. "Toggle"
    local cmdRefActivate = cmdRef .. "Activate"
    local cmdRefDeactivate = cmdRef .. "Deactivate"
    local macroTitle = titlePrefix .. "Activate/De-Activate (Toggle)"
    log(
        string.format(
            "Adding commands %s, %s, %s and macro '%s' (activated: %s)",
            cmdRefToggle,
            cmdRefActivate,
            cmdRefDeactivate,
            macroTitle,
            macroActivated
        )
    )

    create_command(cmdRefToggle, titlePrefix .. "Toggle", activateCallbackName .. "(not " .. globalStateVariableName .. ")", "", "")
    create_command(cmdRefActivate, titlePrefix .. "Activate", activateCallbackName .. "(true)", "", "")
    create_command(cmdRefDeactivate, titlePrefix .. "De-Activate", activateCallbackName .. "(false)", "", "")
    add_macro(macroTitle, activateCallbackName .. "(true)", activateCallbackName .. "(false)", macroActivated)
end

local function kt2mPerS(kt)
    return kt * 0.514444
end

local function ftPerMin2mPerS(ftPerMin)
    return ftPerMin * 0.00508
end

local function ft2m(ft)
    return ft * 0.3048
end

local function clamp(v, min, max)
    return math.max(
        math.min(v, max),
        min
    )
end

local function lerp(v0, v1, t)
  return (1 - t) * v0 + t * v1
end

local function mapRange(v, minV, maxV, minTarget, maxTarget)
    return lerp(
        minTarget,
        maxTarget,
        (clamp(v, minV, maxV) - minV) / (maxV - minV)
    )
end

local function energy()
    local ias_mPerS = kt2mPerS(XPLMGetDataf(ias_kt_dataref))
    local y_m = XPLMGetDataf(y_dataref)
    local m_kg = 500
    return 0.5 * m_kg * ias_mPerS^2 + m_kg * 9.81 * y_m
end

local function activate(isEnabled)
    varioMeter_isEnabled = isEnabled

    log(
        string.format(
            "enabled: %s",
            varioMeter_isEnabled
        )
    )

    if varioMeter_isEnabled then --
        totalEnergySmoothed_mPerS = 0.
        totalEnergyOwnCalcSmoothed_mPerS = 0.
        integrateBufferIndexTime_s = XPLMGetDataf(time_s_dataref)
    end
end

local function init()
    addToggleMacroAndCommand(
        "flightwusel/Variometer/",
        "Variometer: ",
        "variometer_activate_callback",
        "varioMeter_isEnabled"
    )
    addToggleMacroAndCommand(
        "flightwusel/Variometer/Infobox/",
        "Variometer: Infobox: ",
        "variometerInfobox_activate_callback",
        "varioMeterInfobox_isEnabled"
    )
    do_every_frame("variometer_frame_callback()")
    do_every_draw("variometer_draw_callback()")
end

local function do_frame()
    local period_s = XPLMGetDataf(period_s_dataref)
    if period_s < 0.0001 then
        return
    end

    -- integrator
    local _integratedValue = totalEnergy_mPerS
    time_s = XPLMGetDataf(time_s_dataref)
    if time_s > integrateBufferIndexTime_s + integrateStep_s then
        -- write current value(s) into steps that have passed
        local _advanceSteps = math.floor((time_s - integrateBufferIndexTime_s) / integrateStep_s)

        for _i = 1, _advanceSteps do
            integrateBufferIndex = (integrateBufferIndex % integrateBufferSteps) + 1
            integrateBuffer[integrateBufferIndex] = _integratedValue
        end
        local _integrateBufferEarliestIndex = (integrateBufferIndex % integrateBufferSteps) + 1
        integrateBufferEarliest = integrateBuffer[_integrateBufferEarliestIndex]

        integrateBufferSum = 0. -- sum without the earliest value
        for _i,_v in ipairs(integrateBuffer) do
            if _i ~= _integrateBufferEarliestIndex then
                integrateBufferSum = integrateBufferSum + _v
            end
        end

        integrateBufferIndexTime_s = time_s
    end

    -- update every frame to give smooth impression, lerp()ing the current value in and the earliest value out
    local _integrateStep_fraction = (time_s - integrateBufferIndexTime_s) / integrateStep_s
    integrated_mPerS = (
        integrateBufferEarliest * (1. - _integrateStep_fraction) -- earliest value
        + integrateBufferSum -- sum of all but the earliest
        + _integratedValue * _integrateStep_fraction -- current value
    ) / integrateBufferSteps



    -- vario smooth
    vy_mPerS = XPLMGetDataf(vy_dataref)
    local dVy_mPerS = vy_mPerS - vySmoothed_mPerS
    vySmoothed_mPerS = vySmoothed_mPerS + dVy_mPerS / settings['smoothing'] * period_s

    totalEnergy_mPerS = ftPerMin2mPerS(XPLMGetDataf(totalEnergy_ftPerMin_dataref) * lr_totalEnergy_compensationFactor)
    local dTotalEnergy_mPerS = totalEnergy_mPerS - totalEnergySmoothed_mPerS
    totalEnergySmoothed_mPerS = totalEnergySmoothed_mPerS + dTotalEnergy_mPerS / settings['smoothing'] * period_s

    -- own calc
    --y_m = XPLMGetDataf(y_dataref)
    y_m = ft2m(XPLMGetDatad(pressureAlt_dataref))
    -- I don't know why it includes so much stick thermals when using IAS
    --ias_mPerS = kt2mPerS(XPLMGetDataf(ias_kt_dataref))
    ias_mPerS = XPLMGetDataf(tas_mPerS_dataref)
    local g_mPerSSquare = 9.81
    totalEnergyOwnCalc_mPerS = (y_m - (yLast_m or y_m) + (ias_mPerS^2 - (iasLast_mPerS or ias_mPerS)^2) / (2 * g_mPerSSquare)) / period_s

    local dTotalEnergyOwnCalc_mPerS = totalEnergyOwnCalc_mPerS - totalEnergyOwnCalcSmoothed_mPerS
    totalEnergyOwnCalcSmoothed_mPerS = totalEnergyOwnCalcSmoothed_mPerS + dTotalEnergyOwnCalc_mPerS / settings['smoothing'] * period_s

    vyOwn_mPerS = (y_m - (yLast_m or y_m)) / period_s

    yLast_m = y_m
    iasLast_mPerS = ias_mPerS
end

--[[ global ]]
varioMeter_isEnabled = false
varioMeterInfobox_isEnabled = false

function variometer_activate_callback(isEnabled)
    if isEnabled == varioMeter_isEnabled then
        return
    end
    activate(isEnabled)
end

function variometerInfobox_activate_callback(isEnabled)
    varioMeterInfobox_isEnabled = isEnabled
    if varioMeterInfobox_isEnabled and not varioMeter_isEnabled then
        activate(true)
    end
end

function variometer_frame_callback()
    if not varioMeter_isEnabled then
        return
    end

    do_frame()
end

function variometer_draw_callback()
    if not varioMeter_isEnabled then
        return
    end

    local outerTracerWidth = 12
    local radius = 35.
    local x, y = SCREEN_WIDTH - radius * .6 - outerTracerWidth, SCREEN_HEIGHT - radius - outerTracerWidth + 4
    local minValue, maxValue = -5., 5.
    local tickStep = 1.
    local min_deg, max_deg = -225., 45.

    local centerText = string.format('%+.1f m/s', integrated_mPerS)
    local centerTextWidth = measure_string(centerText)

    graphics.set_color(0, 0, 0, .2)
    local _xMin, _yMin = graphics.move_angle(x, y, min_deg, 12)
    local _xMax, _yMax = graphics.move_angle(x, y, max_deg, 12)
    graphics.set_width(4.)
    graphics.draw_triangle(x, y, _xMax, _yMax, _xMin, _yMin)
    --graphics.draw_filled_circle(x, y, radius + 10)
    graphics.draw_filled_arc(x, y, min_deg, max_deg, radius)
--     graphics.draw_filled_circle(x, y, radius)
    graphics.draw_rectangle(_xMin, _yMin, x + radius - 3, _yMax)
    graphics.set_color(1, 1, 1, 0.6)
    --graphics.draw_tick_mark(x, y, mapRange(0, minValue, maxValue, min_deg, max_deg), radius, -5, 1)
    graphics.draw_tick_mark(x, y, mapRange(0, minValue, maxValue, min_deg, max_deg), radius, 10, 2)
    graphics.set_color(1, 1, 1, 0.8)
    for v=minValue, maxValue, tickStep do
        if v ~= 0. then
            graphics.draw_tick_mark(x, y, mapRange(v, minValue, maxValue, min_deg, max_deg), radius, 5, 1)
        end
    end
    graphics.set_color(1, 1, 1, 0.3)
    graphics.draw_arc_line(x, y, min_deg, max_deg, radius, 1.)
    graphics.set_color(0, 1, 1, 0.5)
    graphics.draw_outer_tracer(x, y, mapRange(totalEnergySmoothed_mPerS, minValue, maxValue, min_deg, max_deg), radius, outerTracerWidth)
    --graphics.draw_angle_arrow(x, y, mapRange(totalEnergy_mPerS, minValue, maxValue, min_deg, max_deg), radius, 6, 1.)
    --graphics.draw_inner_tracer(x, y, mapRange(totalEnergy_mPerS, minValue, maxValue, min_deg, max_deg), radius, 6)
    graphics.set_color(1, 0, 1, 0.5)
    graphics.draw_inner_tracer(x, y, mapRange(integrated_mPerS, minValue, maxValue, min_deg, max_deg), radius, 10)

    graphics.set_color(1, 0, 1)
    draw_string(x + radius - centerTextWidth - 5, y - 3, centerText, 255, 255, 255)

    if varioMeterInfobox_isEnabled then
        bubble(
            SCREEN_WIDTH,
            -10,
            string.format(
                '%+.1f m/s TE (own calc smooth)',
                totalEnergyOwnCalcSmoothed_mPerS
            ),
            string.format(
                '%+.1f m/s TE (smooth) (LR corrected * 2)',
                totalEnergySmoothed_mPerS
            ),
            string.format(
                '%+.2f m/s TE (own calc)',
                totalEnergyOwnCalc_mPerS
            ),
            string.format(
                '%+.2f m/s TE (LR corrected * 2)',
                totalEnergy_mPerS
            ),
            string.format(
                '%+.2f m/s (LR)',
                vy_mPerS
            ),
            string.format(
                '%+.2f m/s (own)',
                vyOwn_mPerS
            ),
    --         string.format(
    --             '%+.1f m/s (smooth)',
    --             vySmoothed_mPerS
    --         ),
            string.format(
                '%+.2f m/s TE (own calc integrated %ds)',
                integrated_mPerS,
                integrateDuration_s
            )
        )
    end
end

init()
