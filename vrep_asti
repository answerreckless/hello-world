if (sim_call_type==sim_childscriptcall_initialization) then 
    asti=simGetObjectHandle("Asti")
    lFoot=simGetObjectHandle("leftFootTarget")
    rFoot=simGetObjectHandle("rightFootTarget")
    lPath=simGetObjectHandle("leftFootPath")
    rPath=simGetObjectHandle("rightFootPath")
    lPathLength=simGetPathLength(lPath)
    rPathLength=simGetPathLength(rPath)
    ui=simGetUIHandle("astiUserInterface")
    simSetUIButtonLabel(ui,0,simGetObjectName(asti).." user interface")
    dist=0
    correction=0.0305
    
    minVal={0,            -- Step size
            0,            -- Walking speed
            -math.pi/2,    -- Neck 1
            -math.pi/8,    -- Neck 2
            -math.pi/2,    -- Left shoulder 1
            0,            -- Left shoulder 2
            -math.pi/2,    -- Left forearm
            -math.pi/2,    -- Right shoulder 1
            0,            -- Right shoulder 2
            -math.pi/2}    -- Right forearm
    rangeVal={    2,            -- Step size
                0.8,        -- Walking speed
                math.pi,    -- Neck 1
                math.pi/4,    -- Neck 2
                math.pi/2,    -- Left shoulder 1
                math.pi/2,    -- Left shoulder 2
                math.pi/2,    -- Left forearm
                math.pi/2,    -- Right shoulder 1
                math.pi/2,    -- Right shoulder 2
                math.pi/2}    -- Right forearm
    uiSliderIDs={3,4,5,6,7,8,9,10,11,12}

    relativeStepSize=1
    nominalVelocity=0.4
    neckJoints={simGetObjectHandle("neckJoint0"),simGetObjectHandle("neckJoint1")}
    leftArmJoints={simGetObjectHandle("leftArmJoint0"),simGetObjectHandle("leftArmJoint1"),simGetObjectHandle("leftArmJoint2")}
    rightArmJoints={simGetObjectHandle("rightArmJoint0"),simGetObjectHandle("rightArmJoint1"),simGetObjectHandle("rightArmJoint2")}
        
    -- Now apply current values to the user interface:
    simSetUISlider(ui,uiSliderIDs[1],(relativeStepSize-minVal[1])*1000/rangeVal[1])
    simSetUISlider(ui,uiSliderIDs[2],(nominalVelocity-minVal[2])*1000/rangeVal[2])
    simSetUISlider(ui,uiSliderIDs[3],(simGetJointPosition(neckJoints[1])-minVal[3])*1000/rangeVal[3])
    simSetUISlider(ui,uiSliderIDs[4],(simGetJointPosition(neckJoints[2])-minVal[4])*1000/rangeVal[4])
    simSetUISlider(ui,uiSliderIDs[5],(simGetJointPosition(leftArmJoints[1])-minVal[5])*1000/rangeVal[5])
    simSetUISlider(ui,uiSliderIDs[6],(simGetJointPosition(leftArmJoints[2])-minVal[6])*1000/rangeVal[6])
    simSetUISlider(ui,uiSliderIDs[7],(simGetJointPosition(leftArmJoints[3])-minVal[7])*1000/rangeVal[7])
    simSetUISlider(ui,uiSliderIDs[8],(simGetJointPosition(rightArmJoints[1])-minVal[8])*1000/rangeVal[8])
    simSetUISlider(ui,uiSliderIDs[9],(simGetJointPosition(rightArmJoints[2])-minVal[9])*1000/rangeVal[9])
    simSetUISlider(ui,uiSliderIDs[10],(simGetJointPosition(rightArmJoints[3])-minVal[10])*1000/rangeVal[10])
end 

if (sim_call_type==sim_childscriptcall_cleanup) then 
 
end 

if (sim_call_type==sim_childscriptcall_actuation) then 
    -- Read desired values from the user interface:
    relativeStepSize=minVal[1]+simGetUISlider(ui,uiSliderIDs[1])*rangeVal[1]/1000
    nominalVelocity=minVal[2]+simGetUISlider(ui,uiSliderIDs[2])*rangeVal[2]/1000
    simSetJointTargetPosition(neckJoints[1],minVal[3]+simGetUISlider(ui,uiSliderIDs[3])*rangeVal[3]/1000)
    simSetJointTargetPosition(neckJoints[2],minVal[4]+simGetUISlider(ui,uiSliderIDs[4])*rangeVal[4]/1000)
    simSetJointTargetPosition(leftArmJoints[1],minVal[5]+simGetUISlider(ui,uiSliderIDs[5])*rangeVal[5]/1000)
    simSetJointTargetPosition(leftArmJoints[2],minVal[6]+simGetUISlider(ui,uiSliderIDs[6])*rangeVal[6]/1000)
    simSetJointTargetPosition(leftArmJoints[3],minVal[7]+simGetUISlider(ui,uiSliderIDs[7])*rangeVal[7]/1000)
    simSetJointTargetPosition(rightArmJoints[1],minVal[8]+simGetUISlider(ui,uiSliderIDs[8])*rangeVal[8]/1000)
    simSetJointTargetPosition(rightArmJoints[2],minVal[9]+simGetUISlider(ui,uiSliderIDs[9])*rangeVal[9]/1000)
    simSetJointTargetPosition(rightArmJoints[3],minVal[10]+simGetUISlider(ui,uiSliderIDs[10])*rangeVal[10]/1000)
    
    
    -- Get the desired position and orientation of each foot from the paths (you can also use a table of values for that):
    t=simGetSimulationTimeStep()*nominalVelocity
    dist=dist+t
    lPos=simGetPositionOnPath(lPath,dist/lPathLength)
    lOr=simGetOrientationOnPath(lPath,dist/lPathLength)
    
    p=simGetPathPosition(rPath)
    rPos=simGetPositionOnPath(rPath,(dist+correction)/rPathLength)
    rOr=simGetOrientationOnPath(rPath,(dist+correction)/rPathLength)
    
    
    -- 1 Now we have the desired absolute position and orientation for each foot.
    -- 2 Now transform the absolute position/orientation to position/orientation relative to asimo
    -- 3 Then modulate the movement forward/backward with the desired "step size"
    -- 4 Then transform back into absolute position/orientation:

    -- 1 Now we have the desired absolute position and orientation for each foot.
    -- 2 Now transform the absolute position/orientation to position/orientation relative to asimo
    -- 3 Then modulate the movement forward/backward with the desired "step size"
    -- 4 Then transform back into absolute position/orientation:

    -- 1
    astiM=simGetObjectMatrix(asti,-1) -- -1 to retrieve the absolute worldframe asimo matrix

    -- 2
    astiMInverse=simGetInvertedMatrix(astiM) -- pointer to 12 simFloat values (the last row of the 4x4 matrix (0,0,0,1) is not needed)
                                             -- The x-axis of the orientation component is (matrix[1],matrix[5],matrix[9])
                                             -- The y-axis of the orientation component is (matrix[2],matrix[6],matrix[10])
                                             -- The z-axis of the orientation component is (matrix[3],matrix[7],matrix[11])
                                             -- The translation component is (matrix[4],matrix[8],matrix[12])
    m=simMultiplyMatrices(astiMInverse,simBuildMatrix(lPos,lOr)) -- m: Position Orientation relative to asimo ownframe
                                                                 -- (lPos,lOr): absolute Position Orientation worldframe
    m[8]=m[8]*relativeStepSize -- m[8]: y axis step relative to asimo ownframe

    -- 3 
    m=simMultiplyMatrices(astiM,m) -- m: absolute Position Orientation worldframe
    lPos={m[4],m[8],m[12]}
    lOr=simGetEulerAnglesFromMatrix(m)
    
    m=simMultiplyMatrices(astiMInverse,simBuildMatrix(rPos,rOr))
    m[8]=m[8]*relativeStepSize    
    m=simMultiplyMatrices(astiM,m)
    rPos={m[4],m[8],m[12]}
    rOr=simGetEulerAnglesFromMatrix(m)

    
    
    -- Finally apply the desired positions/orientations to each foot
    -- We simply apply them to two dummy objects that are then handled
    -- by the IK module to automatically calculate all leg joint desired values
    -- Since the leg joints operate in hybrid mode, the IK calculation results
    -- are then automatically applied as the desired values during dynamics calculation
    simSetObjectPosition(lFoot,-1,lPos)
    simSetObjectOrientation(lFoot,-1,lOr)
    
    simSetObjectPosition(rFoot,-1,rPos)
    simSetObjectOrientation(rFoot,-1,rOr)
    
end 
