@0xbf5147b8d6b44672;

struct RobotState {
    oTEe @0 :List(Float64);  # 16 elements (4x4 matrix)
    oTEeD @1 :List(Float64);  # 16 elements (4x4 matrix)
    fTEe @2 :List(Float64);  # 16 elements (4x4 matrix)
    fTNe @3 :List(Float64);  # 16 elements (4x4 matrix)
    neTEe @4 :List(Float64);  # 16 elements (4x4 matrix)
    eeTK @5 :List(Float64);  # 16 elements (4x4 matrix)
    mEe @6 :Float64;
    iEe @7 :List(Float64);  # 9 elements (3x3 matrix)
    fXCee @8 :List(Float64);  # 3 elements
    mLoad @9 :Float64;
    iLoad @10 :List(Float64);  # 9 elements (3x3 matrix)
    fXCload @11 :List(Float64);  # 3 elements
    mTotal @12 :Float64;
    iTotal @13 :List(Float64);  # 9 elements (3x3 matrix)
    fXCtotal @14 :List(Float64);  # 3 elements
    elbow @15 :List(Float64);  # 2 elements
    elbowD @16 :List(Float64);  # 2 elements
    elbowC @17 :List(Float64);  # 2 elements
    delbowC @18 :List(Float64);  # 2 elements
    ddelbowC @19 :List(Float64);  # 2 elements
    tauJ @20 :List(Float64);  # 7 elements
    tauJD @21 :List(Float64);  # 7 elements
    dtauJ @22 :List(Float64);  # 7 elements
    q @23 :List(Float64);  # 7 elements
    qD @24 :List(Float64);  # 7 elements
    dq @25 :List(Float64);  # 7 elements
    dqD @26 :List(Float64);  # 7 elements
    ddqD @27 :List(Float64);  # 7 elements
    jointContact @28 :List(Bool);  # 7 elements
    cartesianContact @29 :List(Bool);  # 6 elements
    jointCollision @30 :List(Bool);  # 7 elements
    cartesianCollision @31 :List(Bool);  # 6 elements
    tauExtHatFiltered @32 :List(Float64);  # 7 elements
    oFExtHatK @33 :List(Float64);  # 6 elements
    kFExtHatK @34 :List(Float64);  # 6 elements
    oDpEeD @35 :List(Float64);  # 6 elements
    oTEeC @36 :List(Float64);  # 16 elements (4x4 matrix)
    oDpEeC @37 :List(Float64);  # 6 elements
    oDdpEeC @38 :List(Float64);  # 6 elements
    theta @39 :List(Float64);  # 7 elements
    dtheta @40 :List(Float64);  # 7 elements
    currentErrors @41 :Text;
    lastMotionErrors @42 :Text;
    controlCommandSuccessRate @43 :Float64;
}

struct GripperState {
    width @0 :Float64;  # Current width in meters
    maxWidth @1 :Float64;  # Maximum width in meters
    isGrasped @2 :Bool;  # True if object is grasped
    temperature @3 :UInt16;  # Temperature in degrees Celsius
    timeStamp @4 :Float64;  # Time since last update
}

struct VacuumGripperState {
    inControlRange @0 :Bool;  # True if vacuum is within control range
    partDetached @1 :Bool;  # True if part is detached
    partPresent @2 :Bool;  # True if part is present
    deviceStatus @3 :UInt8;  # Device status code
    actualPower @4 :Float64;  # Actual power in watts
    vacuum @5 :Float64;  # Current vacuum level
    time @6 :Float64;  # Time since last update
}

struct JointTrajectoryPoint {
    positions @0 :List(Float64);  # 7 joint positions
}

interface RPCService {
    # Robot initialization and recovery
    initializeRobot @0 (ipAddress :Text) -> (result :Void);
    initializeGripper @17 () -> (result :Void);
    initializeVacuumGripper @18 () -> (result :Void);
    automaticErrorRecovery @1 (number :Void) -> (result :Void);
    
    # Robot state
    getRobotState @2 () -> (state :RobotState);
    getJointPoses @3 () -> (poses :List(List(Float64)));  # Each inner list should have 16 elements (10 4x4 matrices in row-major format)
    
    # Joint motion control
    jointPointToPointMotion @4 (targetConfiguration :List(Float64), speedFactor :Float64) -> (result :Bool);
    jointTrajectoryMotion @5 (trajectory :List(JointTrajectoryPoint)) -> (result :Bool);
    
    # Gripper control
    getGripperState @6 () -> (state :GripperState);
    gripperGrasp @7 (width :Float64, speed :Float64, force :Float64, epsilonInner :Float64, epsilonOuter :Float64) -> (success :Bool);
    gripperHoming @8 () -> (success :Bool);
    gripperMove @9 (width :Float64, speed :Float64) -> (success :Bool);
    gripperStop @10 () -> (success :Bool);

    # Collision behavior
    setCollisionBehavior @11 (
        lowerTorqueThresholdsAcceleration :List(Float64),  # 7 elements
        upperTorqueThresholdsAcceleration :List(Float64),  # 7 elements
        lowerTorqueThresholdsNominal :List(Float64),       # 7 elements
        upperTorqueThresholdsNominal :List(Float64),       # 7 elements
        lowerForceThresholdsAcceleration :List(Float64),   # 6 elements
        upperForceThresholdsAcceleration :List(Float64),   # 6 elements
        lowerForceThresholdsNominal :List(Float64),        # 6 elements
        upperForceThresholdsNominal :List(Float64)         # 6 elements
    ) -> (success :Bool);

    # Load inertia
    setLoadInertia @12 (
        mass :Float64,                # Mass in kg
        centerOfMass :List(Float64),  # Center of mass [x, y, z] in meters (3 elements)
        loadInertia :List(Float64)    # Load inertia matrix in row-major format (9 elements)
    ) -> (success :Bool);

    # Vacuum Gripper control
    getVacuumGripperState @13 () -> (state :VacuumGripperState);
    vacuumGripperVacuum @14 (controlPoint :UInt8, timeout :UInt32, profile :UInt8) -> (success :Bool);
    vacuumGripperDropOff @15 (timeout :UInt32) -> (success :Bool);
    vacuumGripperStop @16 () -> (success :Bool);
}