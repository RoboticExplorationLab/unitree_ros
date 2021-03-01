"""
caution! when including this module in julia,
and robot = RobotInterface() is called,
the robot will receive a initialize command and fall down in place

"""



module A1Robot
  using CxxWrap
  using StaticArrays
  using LinearAlgebra
  using ModernRoboticsBook
  # using RobotOS

  # @rosimport sensor_msgs.msg: Imu, Joy
  # @rosimport nav_msgs.msg: Odometry
  # @rosimport geometry_msgs.msg: Pose, Twist, QuaternionStamped, WrenchStamped
  # @rosimport unitree_legged_msgs.msg: MotorCmd, MotorState
  # rostypegen(@__MODULE__)

  # using .sensor_msgs.msg
  # using .nav_msgs.msg
  # using .geometry_msgs.msg
  # using .unitree_legged_msgs.msg

  # define datastructure
  # /home/biorobotics/Documents/unitree_legged_sdk/include/unitree_legged_sdk/comm.h
  mutable struct Cartesian
    x::Float32
    y::Float32
    z::Float32
    Cartesian() = new()
  end

  mutable struct IMU
    quaternion::Array{Float64,1}
    gyroscope::Array{Float64,1}
    accelerometer::Array{Float64,1}
    rpy::Array{Float64,1}
    temperature::Int8
    IMU() = new(zeros(4),zeros(3),zeros(3),zeros(3),0)
  end


  mutable struct Joy
    axes::Array{Float32,1}
    buttons::Array{Int32,1}
    Joy() = new(zeros(Float32,8),zeros(Int32,11))
  end

  mutable struct BaseState
    position::Array{Float64,1}
    velocity::Array{Float64,1}
    orientation::Array{Float64,1}
    BaseState() = new(zeros(Float64,3),zeros(Float64,3),zeros(Float64,4))
  end


  mutable struct LED
    r::UInt8
    g::UInt8
    b::UInt8
    function LED()
        new(0,0,0)
    end
  end
  mutable struct MotorState
    mode::UInt8
    q::Float32
    dq::Float32
    ddq::Float32
    tauEst::Float32
    q_raw::Float32
    dq_raw::Float32
    ddq_raw::Float32
    temperature::Int8
    reserve::SVector{2,UInt32}
    MotorState() = new()
  end

  mutable struct MotorCmd
    mode::UInt8
    q::Float32
    dq::Float32
    tau::Float32
    Kp::Float32
    Kd::Float32
    reserve::SVector{3,UInt32}
    function MotorCmd()
        new(0,0.0,0.0,0.0,0.0,0.0,zeros(SVector{3,UInt32}))
    end
  end

  # this is not identical to unitree_sdk's datastructure
  mutable struct LowState
    # levelFlag::UInt8
    # commVersion::UInt16
    # robotID::UInt16
    # SN::UInt32
    # bandWidth::UInt8
    imu::IMU
    motorState::Array{MotorState,1}
    footForce::Array{Int16,1}
    # footForceEst::SVector{4,Int16}
    # tick::UInt32
    # wirelessRemote::SVector{40,UInt8}
    # reserve::UInt32
    # crc::UInt32
    function LowState()
        # new(0,0,0,0,0,IMU(),@SVector[MotorState() for i in 1:20], zeros(SVector{4,Int16}), zeros(SVector{4,Int16}),0, zeros(SVector{40,UInt8}),0,0)
        new(IMU(), [MotorState() for i in 1:12], zeros(Int16,4))
      end
  end

  # mutable struct LowCmd
  #   levelFlag::UInt8
  #   commVersion::UInt16
  #   robotID::UInt16
  #   SN::UInt32
  #   bandWidth::UInt8
  #   motorCmd::SVector{20,MotorCmd}
  #   led::SVector{4,LED}
  #   wirelessRemote::SVector{40,UInt8}
  #   reserve::UInt32
  #   crc::UInt32
  #   function LowCmd()
  #       new(0,0,0,0,0,@SVector[MotorCmd() for i in 1:20], @SVector[LED() for i in 1:4], zeros(SVector{40,UInt8}),0,0)
  #   end
  # end

  # C_ means these indices are for C++
  const C_FR_ = 0
  const C_FL_ = 1
  const C_RR_ = 2
  const C_RL_ = 3

  const C_FR_0 = 0;
  const C_FR_1 = 1;
  const C_FR_2 = 2;

  const C_FL_0 = 3;
  const C_FL_1 = 4;
  const C_FL_2 = 5;

  const C_RR_0 = 6;
  const C_RR_1 = 7;
  const C_RR_2 = 8;

  const C_RL_0 = 9;
  const C_RL_1 = 10;
  const C_RL_2 = 11;


  # leg indices for Julia
  const FR_ = 1
  const FL_ = 2
  const RR_ = 3
  const RL_ = 4

  const FR_0 = 1;
  const FR_1 = 2;
  const FR_2 = 3;

  const FL_0 = 4;
  const FL_1 = 5;
  const FL_2 = 6;

  const RR_0 = 7;
  const RR_1 = 8;
  const RR_2 = 9;

  const RL_0 = 10;
  const RL_1 = 11;
  const RL_2 = 12;

  @wrapmodule("/home/chiyen/Dev/a1_ws/build/unitree_legged_real/lib/libjulia_a1_interface.so")

  function __init__()
    @initcxx
  end

  function getFbkState!(itf::RobotInterface, fbk_state::LowState)
    fbk_state.footForce[1]= getFootForce(itf, C_FR_)
    fbk_state.footForce[2]= getFootForce(itf, C_FL_)
    fbk_state.footForce[3]= getFootForce(itf, C_RR_)
    fbk_state.footForce[4]= getFootForce(itf, C_RL_)

    for i=1:12
      fbk_state.motorState[i].q  = getMotorStateQ(itf,i-1)
      fbk_state.motorState[i].dq  = getMotorStateDQ(itf,i-1)
      fbk_state.motorState[i].ddq  = getMotorStateDDQ(itf,i-1)
      fbk_state.motorState[i].tauEst  = getMotorStateTau(itf,i-1)
    end
    fbk_state.imu = getIMU(itf)
  end


  """ put robot kinematics, dynamics, fk, jacobian here , may need to move them around """
  # data from https://github.com/unitreerobotics/unitree_ros/blob/master/robots/a1_description/xacro/const.xacro
  const thigh_offset = 0.0838
  const leg_offset_x = 0.1805
  const leg_offset_y = 0.047
  const thigh_length = 0.2

  const hip_mass    =   0.696
  const hip_com_x   =  -0.003311
  const hip_com_y   =   0.000635
  const hip_com_z   =   0.000031
  const hip_ixx     =   0.000469246
  const hip_ixy     =  -0.000009409
  const hip_ixz     =  -0.000000342
  const hip_iyy     =   0.000807490
  const hip_iyz     =  -0.000000466
  const hip_izz     =   0.000552929

  const thigh_mass  =  1.013
  const thigh_com_x = -0.003237
  const thigh_com_y = -0.022327
  const thigh_com_z = -0.027326
  const thigh_ixx   =  0.005529065
  const thigh_ixy   =  0.000004825
  const thigh_ixz   =  0.000343869
  const thigh_iyy   =  0.005139339
  const thigh_iyz   =  0.000022448
  const thigh_izz   =  0.001367788

  const calf_mass   =  0.166
  const calf_com_x  =  0.006435
  const calf_com_y  =  0.0
  const calf_com_z  = -0.107388
  const calf_ixx    =  0.002997972
  const calf_ixy    =  0.0
  const calf_ixz    = -0.000141163
  const calf_iyy    =  0.003014022
  const calf_iyz    =  0.0
  const calf_izz    =  0.000032426

  # get the fk of the leg
  # idx is the index of the leg (we use 0-3)
  # q is the value of joint angles
  function fk(idx::Int, q::Array{Float64,1})
    # these two variables indicates the quadrant of the leg
    front_hind = 1
    mirror = -1
    if idx == C_FR_
      front_hind = 1
      mirror = -1
    elseif idx == C_FL_
      front_hind = 1
      mirror = 1
    elseif idx == C_RR_
      front_hind = -1
      mirror = -1
    else
      front_hind = -1
      mirror = 1
    end
    l = thigh_length
    q1=q[1]; q2=q[2]; q3=q[3];
    p = zeros(3)
    p[1] = leg_offset_x*front_hind - l*sin(q2 + q3) - l*sin(q2)

    p[2] = sin(q1)*(l*cos(q2 + q3) + l*cos(q2)) + thigh_offset*mirror*cos(q1) +leg_offset_y*mirror
    p[3] =  - cos(q1)*(l*cos(q2 + q3) + l*cos(q2)) + thigh_offset*mirror*sin(q1)

    return p
  end

  # the leg jacobian.
  function J(idx::Int, q::Array{Float64,1})
    # these two variables indicates the quadrant of the leg
    # when I derive J in matlab I have right leg define this d as possitive
    d = thigh_offset
    if idx == C_FR_
      front_hind = 1
      mirror = -1
    elseif idx == C_FL_
      front_hind = 1
      mirror = 1
      d = -thigh_offset # left leg has negative d
    elseif idx == C_RR_
      front_hind = -1
      mirror = -1
    else
      front_hind = -1
      mirror = 1
      d = -thigh_offset # left leg has negative d
    end
    l = thigh_length
    q1=q[1]; q2=q[2]; q3=q[3];
    J = zeros(3,3)
    #J[1,1] = 0
    J[1,2] = -l*(cos(q2 + q3) + cos(q2))
    J[1,3] = -l*cos(q2 + q3)
    J[2,1] = cos(q1)*(l*cos(q2 + q3) + l*cos(q2)) + d*sin(q1)
    J[2,2] = -l*sin(q1)*(sin(q2 + q3) + sin(q2))
    J[2,3] = -l*sin(q2 + q3)*sin(q1)
    J[3,1] = sin(q1)*(l*cos(q2 + q3) + l*cos(q2)) - d*cos(q1)
    J[3,2] = l*cos(q1)*(sin(q2 + q3) + sin(q2))
    J[3,3] = l*sin(q2 + q3)*cos(q1)
    return J
  end

  # the derivative of the leg jacobian.
  function dJ(idx::Int, q::Array{Float64,1}, dq::Array{Float64,1})
    # these two variables indicates the quadrant of the leg
    # when I derive J in matlab I have right leg define this d as possitive
    d = thigh_offset
    if idx == C_FR_
      front_hind = 1
      mirror = -1
    elseif idx == C_FL_
      front_hind = 1
      mirror = 1
      d = -thigh_offset # left leg has negative d
    elseif idx == C_RR_
      front_hind = -1
      mirror = -1
    else
      front_hind = -1
      mirror = 1
      d = -thigh_offset # left leg has negative d
    end
    l = thigh_length
    q1=q[1]; q2=q[2]; q3=q[3];
    dq1=dq[1]; dq2=dq[2]; dq3=dq[3];
    dJ = zeros(3,3)
    #J[1,1] = 0
    dJ[1,2] = l*(sin(q2 + q3)*(dq2 + dq3) + dq2*sin(q2))
    dJ[1,3] = l*sin(q2 + q3)*(dq2 + dq3)
    dJ[2,1] = d*dq1*cos(q1) - dq1*sin(q1)*(l*cos(q2 + q3) + l*cos(q2)) - cos(q1)*(dq2*l*sin(q2) + l*sin(q2 + q3)*(dq2 + dq3))
    dJ[2,2] = - l*sin(q1)*(cos(q2 + q3)*(dq2 + dq3) + dq2*cos(q2)) - dq1*l*cos(q1)*(sin(q2 + q3) + sin(q2))
    dJ[2,3] = - l*cos(q2 + q3)*sin(q1)*(dq2 + dq3) - dq1*l*sin(q2 + q3)*cos(q1)
    dJ[3,1] = dq1*cos(q1)*(l*cos(q2 + q3) + l*cos(q2)) - sin(q1)*(dq2*l*sin(q2) + l*sin(q2 + q3)*(dq2 + dq3)) + d*dq1*sin(q1)
    dJ[3,2] = l*cos(q1)*(cos(q2 + q3)*(dq2 + dq3) + dq2*cos(q2)) - dq1*l*sin(q1)*(sin(q2 + q3) + sin(q2))
    dJ[3,3] = l*cos(q2 + q3)*cos(q1)*(dq2 + dq3) - dq1*l*sin(q2 + q3)*sin(q1)
    return dJ
  end

  # get twist list (Slist), home configuration list (Mlist), inertia list (Glist)
  # these definitions are from "ModernRobotics"

  function getTwistList(idx::Integer)
    # these two variables indicates the quadrant of the leg
    front_hind = 1
    mirror = -1
    if idx == C_FR_
      front_hind = 1
      mirror = -1
    elseif idx == C_FL_
      front_hind = 1
      mirror = 1
    elseif idx == C_RR_
      front_hind = -1
      mirror = -1
    else
      front_hind = -1
      mirror = 1
    end
    w1 = [1;0;0];
    pq1 = [leg_offset_x*front_hind;
           leg_offset_y*mirror;
            0];
    w2 = [0;1;0];
    pq2 = [leg_offset_x*front_hind;
           leg_offset_y*mirror + thigh_offset*mirror;
            0];
    w3 = [0;1;0];
    pq3 = [leg_offset_x*front_hind;
           leg_offset_y*mirror + thigh_offset*mirror;
            -thigh_length];
    pf = [leg_offset_x*front_hind;
          leg_offset_y*mirror + thigh_offset*mirror;
          -2*thigh_length];
    Slist = [[w1;-cross(w1,pq1)] [w2;-cross(w2,pq2)] [w3;-cross(w3,pq3)]];
  end

  function getMList(idx::Integer)
    # these two variables indicates the quadrant of the leg
    front_hind = 1
    mirror = -1
    if idx == C_FR_
      front_hind = 1
      mirror = -1
    elseif idx == C_FL_
      front_hind = 1
      mirror = 1
    elseif idx == C_RR_
      front_hind = -1
      mirror = -1
    else
      front_hind = -1
      mirror = 1
    end

    pq1 = [leg_offset_x*front_hind;
           leg_offset_y*mirror;
            0];
    pq2 = [leg_offset_x*front_hind;
           leg_offset_y*mirror + thigh_offset*mirror;
            0];
    pq3 = [leg_offset_x*front_hind;
           leg_offset_y*mirror + thigh_offset*mirror;
            -thigh_length];
    pf = [leg_offset_x*front_hind;
          leg_offset_y*mirror + thigh_offset*mirror;
          -2*thigh_length];

    M1 = [I pq1+[hip_com_x*front_hind;hip_com_y*mirror;hip_com_z]; [0 0 0 1]];
    M2 = [I pq2+[thigh_com_x;thigh_com_y*mirror;thigh_com_z]; [0 0 0 1]];
    M3 = [I pq3+[calf_com_x;calf_com_y;calf_com_z]; [0 0 0 1]];
    M4 = [I pf; [0 0 0 1]];

    M01 = M1;
    M12 = inv(M1)*M2;
    M23 = inv(M2)*M3;
    M34 = inv(M3)*M4;

    return [M01, M12, M23, M34]
  end

  function getGList()
    I1 = [hip_ixx        hip_ixy        hip_ixz;
      hip_ixy        hip_iyy        hip_iyz;
      hip_ixz        hip_iyz        hip_izz ];

    G1 = [I1 zeros(3,3);
      zeros(3,3) diagm([hip_mass, hip_mass, hip_mass])];

    I2 = [thigh_ixx    thigh_ixy      thigh_ixx;
      thigh_ixy    thigh_iyy      thigh_iyz;
      thigh_ixz    thigh_iyz      thigh_izz ];
    G2 = [I2 zeros(3,3);
      zeros(3,3) diagm([thigh_mass, thigh_mass, thigh_mass])];

    I3 = [calf_ixx    calf_ixy      calf_ixx;
      calf_ixy    calf_iyy      calf_iyz;
      calf_ixz    calf_iyz      calf_izz ];
    G3 = [I3 zeros(3,3);
      zeros(3,3) diagm([calf_mass, calf_mass, calf_mass])];

    return [G1, G2, G3]
  end

  # this function calls ModernRoboticsBook
  function getSpatialJ(idx::Integer, q::Array{Float64,1})
    SList = getTwistList(idx)
    return ModernRoboticsBook.JacobianSpace(SList, q)
  end

  function getMassMtx(idx::Integer, q::Array{Float64,1})
    MList = getMList(idx)
    SList = getTwistList(idx)
    GList = getGList()
    return ModernRoboticsBook.MassMatrix(q, MList, GList, SList)
  end

  function getVelQuadraticForces(idx::Integer, q::Array{Float64,1}, dq::Array{Float64,1})
    MList = getMList(idx)
    SList = getTwistList(idx)
    GList = getGList()
    return ModernRoboticsBook.VelQuadraticForces(q, dq, MList, GList, SList)
  end

  function getGravityForces(idx::Integer, q::Array{Float64,1})
    MList = getMList(idx)
    SList = getTwistList(idx)
    GList = getGList()
    return ModernRoboticsBook.GravityForces(q, [0; 0; -9.8], MList, GList, SList)
  end

  """ functions to control the robot """
  # Cartesian space torque control
  # input reference position, velocity, acc (probably from a trajectory)
  # input feedback joint angle, joint angular velocity
  # input desired foot force (in body frame)
  # output desired leg joint torque
  function torque_ctrl(leg_ID::Int,
      ref_p::Vector{Float64}, ref_v::Vector{Float64}, ref_a::Vector{Float64},
      q::Vector{Float64}, dq::Vector{Float64}, F::Vector{Float64})::Vector{Float64}

      Jb = J(leg_ID, q)
      dJ = dJ(leg_ID, q, dq)
      p = fk(leg_ID,q)
      v = Jb*dq
      M = getMassMtx(leg_ID,q)
      c = getVelQuadraticForces(leg_ID,q,dq)
      grav = getGravityForces(leg_ID,q)

      Kp = diagm([90;90;90])
      Kd = diagm([15;15;15])

      tau = Jb'*(Kp*(ref_p-p)+Kd*(ref_v-v)) + Jb'*inv(Jb)'*M*inv(Jb)*(ref_a-dJ*dq) + c + grav;
      # add the foot force
      tau = tau + Jb'*F
      return tau
  end

  function stance_torque_ctrl(leg_ID::Int, q::Vector{Float64}, F::Vector{Float64})::Vector{Float64}
      # J = getSpatialJ(leg_ID, q)
      Jb = J(leg_ID, q)
      tau = Jb'*F
      return tau
  end

  # # put data structure in the module
  # fbk_state = LowState()
  # joy_data_axes = zeros(Float32,8)
  # joy_data_buttons = zeros(Int16,11)
  # body_position = zeros(3)
  # body_orientation = zeros(4)
  # body_velocity  = zeros(3)


  # function init_fbk()
  #   fbk_state = LowState()
  #   joy_data_axes = zeros(Float32,8)
  #   joy_data_buttons = zeros(Int16,11)
  #   body_position = zeros(3)
  #   body_orientation = zeros(4)
  #   body_velocity  = zeros(3)
  #   for i=1:4
  #     fbk_state.footForce[i] = 0
  #   end
  #   for i=1:12
  #       fbk_state.motorState[i].q = 0.01
  #       fbk_state.motorState[i].dq = 0.0
  #   end
  #   body_orientation[1] = 1
  # end

end # end of the module


""" test cases for kinematics and dynamics """



# @show doc()
# doc()
# b = RobotInterface()
# must intialize the robot
# InitSend(b)
# fbk_state = LowState()
# @show d = ReceiveObservation(b)
# c = IMU()

# while true
#     sleep(0.01)
#     getFbkState(b, fbk_state)

#     # now fbk_state are filled with most of the sensor data from the robot
#     @show fbk_state.imu.rpy
# end
