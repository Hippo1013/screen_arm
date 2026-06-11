function sysCall_init()
    local logPath = 'D:/face_screen_arm_build/constrain_base_to_desktop_plane.log'
    local log = io.open(logPath, 'w')
    local function write(message)
        log:write(message .. '\n')
        log:flush()
    end

    local ok, err = pcall(function()
        local sim = require('sim')
        local scenePath = 'D:/face_screen_arm_build/face_screen_support_arm_scene.ttt'
        write('loadedScene=' .. scenePath .. '\tresult=' .. tostring(sim.loadScene(scenePath)))

        local function find(aliasWanted)
            local h = sim.getObject('/' .. aliasWanted, {noError = true})
            if h >= 0 then return h end
            local all = sim.getObjectsInTree(sim.handle_scene)
            for i = 1, #all do
                if sim.getObjectAlias(all[i]) == aliasWanted then return all[i] end
            end
            return -1
        end

        local function worldBounds(handle)
            local minx = sim.getObjectFloatParam(handle, sim.objfloatparam_objbbox_min_x)
            local miny = sim.getObjectFloatParam(handle, sim.objfloatparam_objbbox_min_y)
            local minz = sim.getObjectFloatParam(handle, sim.objfloatparam_objbbox_min_z)
            local maxx = sim.getObjectFloatParam(handle, sim.objfloatparam_objbbox_max_x)
            local maxy = sim.getObjectFloatParam(handle, sim.objfloatparam_objbbox_max_y)
            local maxz = sim.getObjectFloatParam(handle, sim.objfloatparam_objbbox_max_z)
            local m = sim.getObjectMatrix(handle, sim.handle_world)
            local b = {min = {math.huge, math.huge, math.huge}, max = {-math.huge, -math.huge, -math.huge}}
            for _, x in ipairs({minx, maxx}) do
                for _, y in ipairs({miny, maxy}) do
                    for _, z in ipairs({minz, maxz}) do
                        local p = sim.multiplyVector(m, {x, y, z})
                        for k = 1, 3 do
                            if p[k] < b.min[k] then b.min[k] = p[k] end
                            if p[k] > b.max[k] then b.max[k] = p[k] end
                        end
                    end
                end
            end
            return b
        end

        local function removeAlias(aliasWanted)
            local h = find(aliasWanted)
            if h >= 0 then sim.removeObjects({h}) end
        end

        local function setJoint(alias, displayValue, prismatic)
            local h = find(alias)
            if h < 0 then error(alias .. ' was not found') end
            local v = prismatic and displayValue / 1000.0 or displayValue * math.pi / 180.0
            sim.setJointPosition(h, v)
            sim.setJointTargetPosition(h, v)
            write(string.format('setJoint=%s displayValue=%.3f', alias, displayValue))
        end

        local desk = find('office_desk_top')
        local base = find('base_link_respondable')
        local baseVisual = find('base_link_visual')
        local model = find('face_screen_support_arm')
        if desk < 0 then error('office_desk_top was not found') end
        if base < 0 then error('base_link_respondable was not found') end

        local deskBounds = worldBounds(desk)
        local deskTopZ = deskBounds.max[3]
        local targetX = 0.30
        local targetY = 0.00

        removeAlias('office_mount_plane')
        local mount = sim.createDummy(0.035)
        sim.setObjectAlias(mount, 'office_mount_plane')
        sim.setObjectParent(mount, desk, true)
        sim.setObjectPosition(mount, sim.handle_world, {targetX, targetY, deskTopZ})
        sim.setObjectOrientation(mount, sim.handle_world, {0, 0, 0})

        -- A very thin visual washer on the desktop marks the exact mounting plane.
        removeAlias('office_mount_contact_disc')
        local disc = sim.createPrimitiveShape(sim.primitiveshape_cylinder, {0.19, 0.19, 0.002})
        sim.setObjectAlias(disc, 'office_mount_contact_disc')
        sim.setObjectParent(disc, mount, true)
        sim.setObjectPosition(disc, sim.handle_world, {targetX, targetY, deskTopZ + 0.001})
        sim.setObjectOrientation(disc, sim.handle_world, {0, 0, 0})
        sim.setShapeColor(disc, nil, sim.colorcomponent_ambient_diffuse, {0.05, 0.05, 0.055})
        pcall(function() sim.setObjectInt32Param(disc, sim.shapeintparam_static, 1) end)
        pcall(function() sim.setObjectInt32Param(disc, sim.shapeintparam_respondable, 0) end)

        -- Move the real root of the imported kinematic chain, then bind it to the mount plane.
        local bb = worldBounds(base)
        local baseCenterX = (bb.min[1] + bb.max[1]) / 2
        local baseCenterY = (bb.min[2] + bb.max[2]) / 2
        local delta = {targetX - baseCenterX, targetY - baseCenterY, deskTopZ - bb.min[3]}
        local p = sim.getObjectPosition(base, sim.handle_world)
        p[1] = p[1] + delta[1]
        p[2] = p[2] + delta[2]
        p[3] = p[3] + delta[3]
        sim.setObjectPosition(base, sim.handle_world, p)
        sim.setObjectOrientation(base, sim.handle_world, {0, 0, 0})
        sim.setObjectParent(base, mount, true)

        -- Keep the imported model marker at the mount plane too, so the scene tree is not misleading.
        if model >= 0 then
            sim.setObjectPosition(model, sim.handle_world, {targetX, targetY, deskTopZ})
            sim.setObjectOrientation(model, sim.handle_world, {0, 0, 0})
            sim.setObjectParent(model, mount, true)
        end

        setJoint('joint1_base_yaw', 0, false)
        setJoint('joint2_shoulder_pitch', -120, false)
        setJoint('joint3_elbow_pitch', 120, false)
        setJoint('joint4_telescopic', 30, true)
        setJoint('joint5_screen_pan', 0, false)
        setJoint('joint6_screen_pitch', 0, false)

        local afterBase = worldBounds(base)
        local afterVisual = baseVisual >= 0 and worldBounds(baseVisual) or nil
        write(string.format('deskTopZ=%.6f', deskTopZ))
        write(string.format('baseMinZ=%.6f', afterBase.min[3]))
        if afterVisual then write(string.format('baseVisualMinZ=%.6f', afterVisual.min[3])) end
        write(string.format('baseCenter=%.6f %.6f', (afterBase.min[1] + afterBase.max[1]) / 2, (afterBase.min[2] + afterBase.max[2]) / 2))
        write('baseParent=' .. tostring(sim.getObjectAlias(sim.getObjectParent(base))))
        write('modelParent=' .. (model >= 0 and tostring(sim.getObjectAlias(sim.getObjectParent(model))) or 'none'))

        sim.saveScene(scenePath)
        write('savedScene=' .. scenePath)
        write('DONE')
    end)

    if not ok then write('ERROR: ' .. tostring(err)) end
    log:close()
    local sim = require('sim')
    sim.quitSimulator()
end
