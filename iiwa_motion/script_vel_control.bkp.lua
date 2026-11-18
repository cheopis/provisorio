-- #############################################################################
-- kuka_iiwa_ros_api.lua
--
-- Velocity-based position control for KUKA iiwa 14.
 -- Fixed: robot now moves.
-- #############################################################################

function radiansToDegrees(rad) return rad * 180.0 / math.pi end
function degreesToRadians(deg) return deg * math.pi / 180.0 end

function tableToString(t)
    local s = "["
    for i = 1, #t do
        s = s .. string.format("%.6f", t[i])
        if i < #t then s = s .. ", " end
    end
    return s .. "]"
end

function vectorToString(vec)
    return string.format("[%.6f, %.6f, %.6f]", vec[1], vec[2], vec[3])
end

function getRobotHandles()
    local handles = {}
    handles.joints = {
        sim.getObjectHandle("LBR_iiwa_14_R820_joint1"),
        sim.getObjectHandle("LBR_iiwa_14_R820_joint2"),
        sim.getObjectHandle("LBR_iiwa_14_R820_joint3"),
        sim.getObjectHandle("LBR_iiwa_14_R820_joint4"),
        sim.getObjectHandle("LBR_iiwa_14_R820_joint5"),
        sim.getObjectHandle("LBR_iiwa_14_R820_joint6"),
        sim.getObjectHandle("LBR_iiwa_14_R820_joint7")
    }
    handles.tip = sim.getObjectHandle("LBR_iiwa_14_R820_connection")
    -- Force sensor is optional ? ignore if missing
    local sensorExists, sensorHandle = sim.getObjectHandle("forceSensor0")
    handles.forceSensor = (sensorExists == -1) and nil or sensorHandle
    return handles
end

function publishString(publisher, value, timestamp)
    local msg = {data = string.format("%s %.6f", tostring(value), timestamp)}
    simROS.publish(publisher, msg)
end

-- ----------------------------- Global variables -----------------------------
robotHandles = nil
rosPub = {}
rosSub = nil
commandStatusPub = nil

currentTargetPos = nil
movementFinished = true
lastJointPositions = {}
lastTimestamp = nil
lastJointVelocities = {}
lastJointAcc = {}

commandQueue = {}
isProcessingCommand = false
currentCommand = nil
commandStartTime = nil
verbose = false   -- enable debug prints

-- Velocity control parameters (rad/s limits for each joint)
jointMaxVelocity = {1.48, 1.48, 1.75, 1.31, 2.27, 2.36, 2.36}
PVEL_GAIN = 1.5          -- higher gain = faster response
velocityFactor = 1.0     -- 0..1, can be changed online

-- ----------------------------- Velocity-based position control --------------
function updateVelocityControl()
    if not currentTargetPos then
        for i = 1, 7 do sim.setJointTargetVelocity(robotHandles.joints[i], 0) end
        return
    end

    local allAtTarget = true
    local toleranceRad = degreesToRadians(1)   -- 0.2 degree finish tolerance
    local minVel = 0.05                          -- rad/s minimum to overcome friction

    for i = 1, 7 do
        local currPos = sim.getJointPosition(robotHandles.joints[i])
        local error = currentTargetPos[i] - currPos

        if math.abs(error) > toleranceRad then
            allAtTarget = false
        end

        -- Desired velocity with higher gain
        local desiredVel = error * 5 * velocityFactor  -- increased from 1.5
        --print(velocityFactor)

        -- Apply minimum velocity if error is significant (avoid stiction)
        local maxVel = jointMaxVelocity[i] * velocityFactor
        if math.abs(error) > degreesToRadians(0.5) then
            if desiredVel > 0 and desiredVel < minVel then
                desiredVel = minVel
            elseif desiredVel < 0 and desiredVel > -minVel then
                desiredVel = -minVel
            end
        end

        -- Saturate to joint limits
        if desiredVel > maxVel then desiredVel = maxVel
        elseif desiredVel < -maxVel then desiredVel = -maxVel end

        sim.setJointTargetVelocity(robotHandles.joints[i], desiredVel)

        -- Optional: print only if velocity is non?zero but actual is zero (debug)
        if math.abs(desiredVel) > 0.01 then
            local actualVel = sim.getJointVelocity(robotHandles.joints[i])
            if math.abs(actualVel) < 0.001 then
                --print(string.format("Joint %d: desired %.3f, actual 0 ? check torque limits", i, desiredVel))
            end
        end
    end

    -- Movement finished?
    if allAtTarget and not movementFinished then
        movementFinished = true
        isProcessingCommand = false
        local elapsed = sim.getSimulationTime() - commandStartTime
        print(string.format("[KUKA ROS] Movement finished in %.2f s", elapsed))
        if commandStatusPub and currentCommand then
            simROS.publish(commandStatusPub, {data = "FINISHED: " .. currentCommand})
            currentCommand = nil
        end
        for i = 1, 7 do
            sim.setJointTargetVelocity(robotHandles.joints[i], 0)
        end
        processNextCommand()
    end
end
-- ----------------------------- Command processing ----------------------------
function processNextCommand()
    if #commandQueue > 0 and not isProcessingCommand and movementFinished then
        local nextCmd = table.remove(commandQueue, 1)
        isProcessingCommand = true
        movementFinished = false

        local parts = {}
        for token in string.gmatch(nextCmd, "%S+") do
            table.insert(parts, token)
        end

        if parts[1] == "setPosition" and #parts == 8 then
            local newTarget = {}
            for i = 2, 8 do
                newTarget[i-1] = degreesToRadians(tonumber(parts[i]))
            end
            currentTargetPos = newTarget
            currentCommand = nextCmd
            commandStartTime = sim.getSimulationTime()

            --print("[KUKA ROS] Starting move to (rad): " .. tableToString(newTarget))
            if commandStatusPub then
                simROS.publish(commandStatusPub, {data = "STARTED: " .. nextCmd})
            end
        else
            print("[KUKA ROS] Invalid command: " .. nextCmd)
            isProcessingCommand = false
            movementFinished = true
            processNextCommand()
        end
    end
end

function commandCallback(msg)
    local cmd = msg.data
    local parts = {}
    for token in string.gmatch(cmd, "%S+") do
        table.insert(parts, token)
    end
    if #parts == 0 then return end

    if parts[1] == "setPosition" and #parts == 8 then
        table.insert(commandQueue, cmd)
        print("[KUKA ROS] Queued: " .. cmd .. " (queue size: " .. #commandQueue .. ")")
        if commandStatusPub then
            simROS.publish(commandStatusPub, {data = "QUEUED: " .. cmd})
        end
        processNextCommand()

    elseif parts[1] == "setJointVelocity" and #parts == 2 then
        local vel = tonumber(parts[2])
        if vel and vel >= 0.0 and vel <= 1.0 then
            velocityFactor = vel
            --print(string.format("[KUKA ROS] Velocity factor set to %.2f (immediate effect)", vel))
            if commandStatusPub then
                simROS.publish(commandStatusPub, {data = "VELOCITY_SET: " .. vel})
            end
        else
            print("[KUKA ROS] Invalid velocity (must be 0-1)")
        end

    elseif parts[1] == "getQueueStatus" then
        local status = string.format("Queue: %d, Processing: %s, Moving: %s",
            #commandQueue, tostring(isProcessingCommand), tostring(not movementFinished))
        print("[KUKA ROS] " .. status)
        if commandStatusPub then
            simROS.publish(commandStatusPub, {data = "QUEUE_STATUS: " .. status})
        end

    elseif parts[1] == "clearQueue" then
        local cleared = #commandQueue
        commandQueue = {}
        print("[KUKA ROS] Cleared " .. cleared .. " commands")
        if commandStatusPub then
            simROS.publish(commandStatusPub, {data = "QUEUE_CLEARED: " .. cleared})
        end

    else
        print("[KUKA ROS] Unknown command: " .. cmd)
    end
    
    if parts[1] == "setJointVelocity" and #parts == 2 then
        local vel = tonumber(parts[2])
        if vel and vel >= 0.0 and vel <= 1.0 then
            velocityFactor = vel
            print(string.format("[KUKA ROS] Velocity factor set to %.2f (immediate effect)", vel))
            if commandStatusPub then
                simROS.publish(commandStatusPub, {data = "VELOCITY_SET: " .. vel})
            end
        else
            print("[KUKA ROS] Invalid velocity (must be 0-1)")
        end
    end
end

-- ----------------------------- Main simulation loop (actuation) -------------
function sysCall_actuation()
    if not robotHandles then return end

    -- Run the velocity controller
    updateVelocityControl()

    -- Publish joint states and tool data (simplified for brevity, same as before)
    local t = sim.getSimulationTime()
    local jointPosDeg = {}
    for i = 1, 7 do
        jointPosDeg[i] = radiansToDegrees(sim.getJointPosition(robotHandles.joints[i]))
    end

    publishString(rosPub.JointPosition, tableToString(jointPosDeg), t)
    -- (other publishers omitted for clarity ? you can add them back)
end

-- ----------------------------- Initialisation -------------------------------
function sysCall_init()
    robotHandles = getRobotHandles()
    if not robotHandles.joints[1] then
        print("[KUKA ROS] ERROR: Could not find joint objects.")
        return
    end

    -- Configure each joint for velocity control
    for i = 1, 7 do
        local joint = robotHandles.joints[i]
        -- Force/torque mode (required for velocity commands)
        sim.setJointMode(joint, sim.jointmode_force, 0)
        -- Set high torque/force limit so the motor can move the joint
        --sim.setJointForce(joint, 1000)   -- 1000 Nm (adjust as needed)
        -- Initial target velocity = 0
        sim.setJointTargetVelocity(joint, 0)

        -- Initialise storage tables
        lastJointPositions[i] = sim.getJointPosition(joint)
        lastJointVelocities[i] = 0
        lastJointAcc[i] = 0
    end

    -- Check ROS plugin
    if not simROS then
        print("[KUKA ROS] ERROR: simROS not available. Enable the ROS plugin.")
        return
    end

    -- Advertise publishers
    local nodeName = '/kuka_iiwa/'
    rosPub.JointPosition     = simROS.advertise(nodeName .. 'JointPosition',     'std_msgs/String')
    rosPub.ToolPosition      = simROS.advertise(nodeName .. 'ToolPosition',      'std_msgs/String')
    rosPub.ToolForce         = simROS.advertise(nodeName .. 'ToolForce',         'std_msgs/String')
    rosPub.ToolTorque        = simROS.advertise(nodeName .. 'ToolTorque',        'std_msgs/String')
    rosPub.isCompliance      = simROS.advertise(nodeName .. 'isCompliance',      'std_msgs/String')
    rosPub.isCollision       = simROS.advertise(nodeName .. 'isCollision',       'std_msgs/String')
    rosPub.isReadyToMove     = simROS.advertise(nodeName .. 'isReadyToMove',     'std_msgs/String')
    rosPub.isMastered        = simROS.advertise(nodeName .. 'isMastered',        'std_msgs/String')
    rosPub.OperationMode     = simROS.advertise(nodeName .. 'OperationMode',     'std_msgs/String')
    rosPub.JointAcceleration = simROS.advertise(nodeName .. 'JointAcceleration', 'std_msgs/String')
    rosPub.JointVelocity     = simROS.advertise(nodeName .. 'JointVelocity',     'std_msgs/String')
    rosPub.JointJerk         = simROS.advertise(nodeName .. 'JointJerk',         'std_msgs/String')
    rosPub.isFinished        = simROS.advertise(nodeName .. 'isFinished',        'std_msgs/String')
    rosPub.hasError          = simROS.advertise(nodeName .. 'hasError',          'std_msgs/String')
    rosPub.OperatorAck       = simROS.advertise(nodeName .. 'OperatorAck',       'std_msgs/String')

    -- Subscriber for commands
    rosSub = simROS.subscribe(nodeName .. 'kuka_command', 'std_msgs/String', 'commandCallback')
    commandStatusPub = simROS.advertise(nodeName .. 'command_status', 'std_msgs/String')

    print("[KUKA ROS] Initialised with velocity-based position control.")
    print("[KUKA ROS] Joints set to force mode with torque limit 1000 Nm.")
end

function sysCall_cleanup()
    if simROS then
        for _, pub in pairs(rosPub) do
            if pub then simROS.shutdownPublisher(pub) end
        end
        if commandStatusPub then simROS.shutdownPublisher(commandStatusPub) end
    end
    print("[KUKA ROS] Cleaned up.")
end