function sysCall_init()
    local logPath = 'D:/face_screen_arm_build/add_office_scene.log'
    local log = io.open(logPath, 'w')
    local function write(message)
        log:write(message .. '\n')
        log:flush()
    end

    local ok, err = pcall(function()
        local sim = require('sim')
        local scenePath = 'D:/face_screen_arm_build/face_screen_support_arm_scene.ttt'
        local modelPath = 'D:/CoppeliaSim_Edu_V4_10_0_rev0/models/robots/non-mobile/face_screen_support_arm.ttm'

        write('loadedScene=' .. scenePath .. '\tresult=' .. tostring(sim.loadScene(scenePath)))

        local function removeByPrefix(prefix)
            local all = sim.getObjectsInTree(sim.handle_scene)
            for i = 1, #all do
                local alias = sim.getObjectAlias(all[i])
                if alias and string.sub(alias, 1, #prefix) == prefix then
                    sim.removeObjects({all[i]})
                end
            end
        end

        removeByPrefix('office_')

        local root = sim.createDummy(0.02)
        sim.setObjectAlias(root, 'office_scene')
        sim.setObjectPosition(root, sim.handle_world, {0, 0, 0})

        local function color(h, rgb)
            sim.setShapeColor(h, nil, sim.colorcomponent_ambient_diffuse, rgb)
        end

        local function makeShape(alias, primitive, size, pos, rgb, respondable)
            local h = sim.createPrimitiveShape(primitive, size)
            sim.setObjectAlias(h, alias)
            sim.setObjectParent(h, root, true)
            sim.setObjectPosition(h, sim.handle_world, pos)
            pcall(function() sim.setObjectInt32Param(h, sim.shapeintparam_static, 1) end)
            pcall(function() sim.setObjectInt32Param(h, sim.shapeintparam_respondable, respondable and 1 or 0) end)
            pcall(function() sim.setBoolProperty(h, 'dynamic', false) end)
            pcall(function() sim.setBoolProperty(h, 'respondable', respondable) end)
            color(h, rgb)
            return h
        end

        local cuboid = sim.primitiveshape_cuboid
        local cylinder = sim.primitiveshape_cylinder
        local spheroid = sim.primitiveshape_spheroid

        -- Desk: 1.40 m wide, 0.75 m deep, 0.74 m high.
        local deskX = 0.35
        local deskY = 0.0
        local deskTopZ = 0.74
        makeShape('office_desk_top', cuboid, {0.75, 1.40, 0.04}, {deskX, deskY, deskTopZ}, {0.62, 0.42, 0.24}, true)
        local legZ = deskTopZ / 2 - 0.02
        for _, p in ipairs({
            {-0.30, -0.62}, {-0.30, 0.62}, {0.70, -0.62}, {0.70, 0.62}
        }) do
            makeShape('office_desk_leg', cuboid, {0.05, 0.05, 0.70}, {p[1], p[2], legZ}, {0.30, 0.24, 0.18}, true)
        end

        -- Chair: realistic simple office chair proportions.
        makeShape('office_chair_seat', cuboid, {0.48, 0.48, 0.06}, {1.05, 0, 0.45}, {0.08, 0.10, 0.12}, true)
        makeShape('office_chair_back', cuboid, {0.08, 0.50, 0.55}, {1.27, 0, 0.75}, {0.08, 0.10, 0.12}, true)
        makeShape('office_chair_post', cylinder, {0.06, 0.06, 0.42}, {1.05, 0, 0.22}, {0.12, 0.12, 0.12}, true)
        makeShape('office_chair_base', cylinder, {0.42, 0.42, 0.03}, {1.05, 0, 0.04}, {0.10, 0.10, 0.10}, true)

        -- Seated person, 1.70-1.75 m standing equivalent, seated head height about 1.25 m.
        makeShape('office_person_pelvis', cuboid, {0.28, 0.34, 0.14}, {0.98, 0, 0.54}, {0.16, 0.24, 0.42}, false)
        makeShape('office_person_torso', cuboid, {0.32, 0.42, 0.48}, {0.94, 0, 0.82}, {0.20, 0.32, 0.58}, false)
        makeShape('office_person_neck', cylinder, {0.06, 0.06, 0.09}, {0.90, 0, 1.10}, {0.80, 0.62, 0.46}, false)
        makeShape('office_person_head', spheroid, {0.18, 0.16, 0.22}, {0.88, 0, 1.25}, {0.82, 0.64, 0.48}, false)
        makeShape('office_face_target', spheroid, {0.035, 0.035, 0.035}, {0.78, 0, 1.26}, {1.0, 0.15, 0.10}, false)

        -- Arms resting near the desk edge.
        makeShape('office_person_left_upper_arm', cuboid, {0.10, 0.08, 0.34}, {0.88, -0.28, 0.83}, {0.20, 0.32, 0.58}, false)
        makeShape('office_person_right_upper_arm', cuboid, {0.10, 0.08, 0.34}, {0.88, 0.28, 0.83}, {0.20, 0.32, 0.58}, false)
        makeShape('office_person_left_forearm', cuboid, {0.34, 0.08, 0.08}, {0.63, -0.28, 0.75}, {0.80, 0.62, 0.46}, false)
        makeShape('office_person_right_forearm', cuboid, {0.34, 0.08, 0.08}, {0.63, 0.28, 0.75}, {0.80, 0.62, 0.46}, false)

        -- Seated legs under the desk.
        makeShape('office_person_left_thigh', cuboid, {0.36, 0.12, 0.10}, {0.78, -0.13, 0.45}, {0.08, 0.10, 0.28}, false)
        makeShape('office_person_right_thigh', cuboid, {0.36, 0.12, 0.10}, {0.78, 0.13, 0.45}, {0.08, 0.10, 0.28}, false)
        makeShape('office_person_left_shin', cuboid, {0.11, 0.10, 0.42}, {0.62, -0.13, 0.24}, {0.08, 0.10, 0.28}, false)
        makeShape('office_person_right_shin', cuboid, {0.11, 0.10, 0.42}, {0.62, 0.13, 0.24}, {0.08, 0.10, 0.28}, false)
        makeShape('office_person_left_foot', cuboid, {0.25, 0.11, 0.05}, {0.50, -0.13, 0.04}, {0.04, 0.04, 0.05}, false)
        makeShape('office_person_right_foot', cuboid, {0.25, 0.11, 0.05}, {0.50, 0.13, 0.04}, {0.04, 0.04, 0.05}, false)

        -- A small monitor/keyboard footprint on the desk gives scale context.
        makeShape('office_keyboard_reference', cuboid, {0.32, 0.12, 0.015}, {0.55, 0, 0.77}, {0.02, 0.02, 0.025}, false)
        makeShape('office_face_target_label_marker', cylinder, {0.01, 0.01, 0.20}, {0.78, 0, 1.14}, {1.0, 0.15, 0.10}, false)

        local modelBase = sim.getObject('/face_screen_support_arm', {noError = true})
        if modelBase and modelBase >= 0 then
            sim.saveModel(modelBase, modelPath)
            write('savedModel=' .. modelPath)
        end
        sim.saveScene(scenePath)
        write('savedScene=' .. scenePath)
        write('officeSceneRoot=' .. tostring(root))
        write('DONE')
    end)

    if not ok then write('ERROR: ' .. tostring(err)) end
    log:close()
    local sim = require('sim')
    sim.quitSimulator()
end
