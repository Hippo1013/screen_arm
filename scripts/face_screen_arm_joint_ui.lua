sim = require 'sim'
simUI = nil

local ui = nil
local suppress = false
local joints = {
    {name = 'joint1_base_yaw', label = 'J1 base yaw', unit = 'deg', lo = -150, hi = 150, step = 1, slider = 101, spin = 201},
    {name = 'joint2_shoulder_pitch', label = 'J2 shoulder', unit = 'deg', lo = -180, hi = 0, step = 1, slider = 102, spin = 202},
    {name = 'joint3_elbow_pitch', label = 'J3 elbow', unit = 'deg', lo = -120, hi = 150, step = 1, slider = 103, spin = 203},
    {name = 'joint4_telescopic', label = 'J4 telescope', unit = 'mm', lo = 0, hi = 280, step = 1, slider = 104, spin = 204},
    {name = 'joint5_screen_pan', label = 'J5 screen pan', unit = 'deg', lo = -180, hi = 180, step = 1, slider = 105, spin = 205},
    {name = 'joint6_screen_pitch', label = 'J6 screen pitch', unit = 'deg', lo = -60, hi = 60, step = 1, slider = 106, spin = 206},
}

local poses = {
    home = {0, -120, 120, 30, 0, 0},
    left = {45, -35, -45, 120, -90, 10},
    right = {-45, -35, -45, 120, 90, 10},
    near = {0, -55, -55, 20, 0, 5},
    far = {0, -20, -20, 420, 0, 0},
}

local function clamp(v, lo, hi)
    if v < lo then return lo end
    if v > hi then return hi end
    return v
end

local function displayToSim(j, v)
    if j.unit == 'deg' then return v * math.pi / 180 end
    return v / 1000
end

local function simToDisplay(j, v)
    if j.unit == 'deg' then return v * 180 / math.pi end
    return v * 1000
end

local function findJoint(name)
    local h = sim.getObject('/' .. name, {noError = true})
    if h >= 0 then return h end
    local all = sim.getObjectsInTree(sim.handle_scene, sim.sceneobject_joint)
    for i = 1, #all do
        if sim.getObjectAlias(all[i]) == name then return all[i] end
    end
    return -1
end

local function setJointDisplayValue(index, value)
    local j = joints[index]
    value = clamp(value, j.lo, j.hi)
    if j.handle and j.handle >= 0 then
        sim.setJointTargetPosition(j.handle, displayToSim(j, value))
        sim.setJointPosition(j.handle, displayToSim(j, value))
    end
    if ui then
        suppress = true
        simUI.setSliderValue(ui, j.slider, value, true)
        simUI.setSpinboxValue(ui, j.spin, value, true)
        suppress = false
    end
end

local function refreshUiFromScene()
    if not ui then return end
    suppress = true
    for i = 1, #joints do
        local j = joints[i]
        if j.handle and j.handle >= 0 then
            local value = clamp(simToDisplay(j, sim.getJointPosition(j.handle)), j.lo, j.hi)
            simUI.setSliderValue(ui, j.slider, value, true)
            simUI.setSpinboxValue(ui, j.spin, value, true)
        end
    end
    suppress = false
end

function jointSliderChanged(uiHandle, id, value)
    if suppress then return end
    for i = 1, #joints do
        if joints[i].slider == id then
            setJointDisplayValue(i, value)
            return
        end
    end
end

function jointSpinChanged(uiHandle, id, value)
    if suppress then return end
    for i = 1, #joints do
        if joints[i].spin == id then
            setJointDisplayValue(i, value)
            return
        end
    end
end

function applyPose(values)
    for i = 1, #joints do setJointDisplayValue(i, values[i]) end
end

function poseHome() applyPose(poses.home) end
function poseLeft() applyPose(poses.left) end
function poseRight() applyPose(poses.right) end
function poseNear() applyPose(poses.near) end
function poseFar() applyPose(poses.far) end

function closeUi()
    if ui then
        simUI.destroy(ui)
        ui = nil
    end
end

local function createUi()
    if ui then return end
    if not simUI then
        local ok, module = pcall(require, 'simUI')
        if not ok then
            sim.addLog(sim.verbosity_warnings, 'Joint UI: simUI is not available in this CoppeliaSim session')
            return
        end
        simUI = module
    end
    local rows = ''
    for i = 1, #joints do
        local j = joints[i]
        rows = rows .. string.format([[
            <label text="%s"/>
            <hslider id="%d" minimum="%d" maximum="%d" on-change="jointSliderChanged"/>
            <spinbox id="%d" minimum="%d" maximum="%d" step="%g" decimals="1" suffix=" %s" on-change="jointSpinChanged"/>
        ]], j.label, j.slider, j.lo, j.hi, j.spin, j.lo, j.hi, j.step, j.unit)
    end
    local xml = [[
        <ui title="Face Screen Arm Control" closeable="true" on-close="closeUi" resizable="false" placement="relative" position="20,20">
            <group layout="grid" flat="true" content-margins="6,6,6,6">
    ]] .. rows .. [[
            </group>
            <group layout="hbox" flat="true" content-margins="6,2,6,6">
                <button text="Home" on-click="poseHome"/>
                <button text="Left" on-click="poseLeft"/>
                <button text="Right" on-click="poseRight"/>
                <button text="Near" on-click="poseNear"/>
                <button text="Far" on-click="poseFar"/>
            </group>
        </ui>
    ]]
    ui = simUI.create(xml)
    refreshUiFromScene()
end

function sysCall_init()
    for i = 1, #joints do
        joints[i].handle = findJoint(joints[i].name)
        if joints[i].handle < 0 then
            sim.addLog(sim.verbosity_warnings, 'Joint UI: missing ' .. joints[i].name)
        end
    end
    createUi()
end

function sysCall_nonSimulation()
    refreshUiFromScene()
end

function sysCall_cleanup()
    closeUi()
end
