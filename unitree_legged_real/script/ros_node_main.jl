#include("init_sim_env.jl")
if ! isinteractive()
    include("init_sim_env.jl")
end

# joy callback
function joy_stick_callback(msg::sensor_msgs.msg.Joy, out_data::A1Robot.Joy)
    for i=1:8
        out_data.axes[i] = msg.axes[i]
    end
    for i=1:11
        out_data.buttons[i] = msg.buttons[i]
    end
end

# base state callback
function sim_pose_callback(msg::nav_msgs.msg.Odometry, base_state::A1Robot.BaseState)
    # pt_msg = Point(msg.point.x, msg.point.y, 0.0)
    base_state.position[1] = msg.pose.pose.position.x
    base_state.position[2] = msg.pose.pose.position.y
    base_state.position[3] = msg.pose.pose.position.z
    base_state.orientation[2] = msg.pose.pose.orientation.x
    base_state.orientation[3] = msg.pose.pose.orientation.y
    base_state.orientation[4] = msg.pose.pose.orientation.z
    base_state.orientation[1] = msg.pose.pose.orientation.w
    base_state.velocity[1] = msg.twist.twist.linear.x
    base_state.velocity[2] = msg.twist.twist.linear.y
    base_state.velocity[3] = msg.twist.twist.linear.z
end

# motor state callbacks
function sim_low_state_callback(msg::unitree_legged_msgs.msg.LowState, state::A1Robot.LowState)
    for i=1:12
        state.motorState[i].q = msg.motorState[i].q
        state.motorState[i].dq = msg.motorState[i].dq
        state.motorState[i].tauEst = msg.motorState[i].tauEst
    end
end

# foot force callback
function sim_foot_force_FR_callback(msg::geometry_msgs.msg.WrenchStamped, state::A1Robot.LowState)
    state.footForce[1] = convert(Int16, floor(msg.wrench.force.z*10))
end
function sim_foot_force_FL_callback(msg::geometry_msgs.msg.WrenchStamped, state::A1Robot.LowState)
    state.footForce[2] = convert(Int16, floor(msg.wrench.force.z*10))
end
function sim_foot_force_RR_callback(msg::geometry_msgs.msg.WrenchStamped, state::A1Robot.LowState)
    state.footForce[3] = convert(Int16, floor(msg.wrench.force.z*10))
end
function sim_foot_force_RL_callback(msg::geometry_msgs.msg.WrenchStamped, state::A1Robot.LowState)
    state.footForce[4] = convert(Int16, floor(msg.wrench.force.z*10))
end

# IMU callback
function sim_imu_callback(msg::sensor_msgs.msg.Imu, state::A1Robot.LowState)
    state.imu.gyroscope[1] = msg.angular_velocity.x
    state.imu.gyroscope[2] = msg.angular_velocity.y
    state.imu.gyroscope[3] = msg.angular_velocity.z
end

""" main entrance """
function main()
    init_node("rosjl_motor_read",disable_signals="True")

    """ all robot feedback subscriber"""
    # joy data and sub
    joy_data = A1Robot.Joy()
    joy_stick_sub = Subscriber{sensor_msgs.msg.Joy}("joy", joy_stick_callback,(joy_data,), queue_size=1)

    # body pose state estimation sub
    base_state = A1Robot.BaseState()
    base_state.orientation[1] = 1.0
    sim_pose_sub = Subscriber{nav_msgs.msg.Odometry}("body_pose_ground_truth", sim_pose_callback, (base_state,), queue_size=1)

    # motor states
    fbk_state = A1Robot.LowState()
    for i=1:12
        fbk_state.motorState[i].q = 0.01
        fbk_state.motorState[i].dq = 0.0
    end
    sim_low_state_sub = Subscriber{unitree_legged_msgs.msg.LowState}("a1_gazebo/lowState/state",
        sim_low_state_callback, (fbk_state,), queue_size=1)
    # sim_joint_FR_hip_sub = Subscriber{unitree_legged_msgs.msg.MotorState}("a1_gazebo/FR_hip_controller/state",
    #     sim_joint_FR_hip_callback, (fbk_state.motorState[1],), queue_size=1)
    # sim_joint_FR_thigh_sub = Subscriber{unitree_legged_msgs.msg.MotorState}("a1_gazebo/FR_thigh_controller/state",
    #     sim_joint_FR_thigh_callback, (fbk_state.motorState[2],), queue_size=1)
    # sim_joint_FR_calf_sub = Subscriber{unitree_legged_msgs.msg.MotorState}("a1_gazebo/FR_calf_controller/state",
    #     sim_joint_FR_calf_callback, (fbk_state.motorState[3],), queue_size=1)
    # sim_joint_FL_hip_sub = Subscriber{unitree_legged_msgs.msg.MotorState}("a1_gazebo/FL_hip_controller/state",
    #     sim_joint_FL_hip_callback, (fbk_state.motorState[4],), queue_size=1)
    # sim_joint_FL_thigh_sub = Subscriber{unitree_legged_msgs.msg.MotorState}("a1_gazebo/FL_thigh_controller/state",
    #     sim_joint_FL_thigh_callback, (fbk_state.motorState[5],), queue_size=1)
    # sim_joint_FL_calf_sub = Subscriber{unitree_legged_msgs.msg.MotorState}("a1_gazebo/FL_calf_controller/state",
    #     sim_joint_FL_calf_callback, (fbk_state.motorState[6],), queue_size=1)

    # sim_joint_RR_hip_sub = Subscriber{unitree_legged_msgs.msg.MotorState}("a1_gazebo/RR_hip_controller/state",
    #     sim_joint_RR_hip_callback, (fbk_state.motorState[7],), queue_size=1)
    # sim_joint_RR_thigh_sub = Subscriber{unitree_legged_msgs.msg.MotorState}("a1_gazebo/RR_thigh_controller/state",
    #     sim_joint_RR_thigh_callback, (fbk_state.motorState[8],), queue_size=1)
    # sim_joint_RR_calf_sub = Subscriber{unitree_legged_msgs.msg.MotorState}("a1_gazebo/RR_calf_controller/state",
    #     sim_joint_RR_calf_callback, (fbk_state.motorState[9],), queue_size=1)
    # sim_joint_RL_hip_sub = Subscriber{unitree_legged_msgs.msg.MotorState}("a1_gazebo/RL_hip_controller/state",
    #     sim_joint_RL_hip_callback, (fbk_state.motorState[10],), queue_size=1)
    # sim_joint_RL_thigh_sub = Subscriber{unitree_legged_msgs.msg.MotorState}("a1_gazebo/RL_thigh_controller/state",
    #     sim_joint_RL_thigh_callback, (fbk_state.motorState[11],), queue_size=1)
    # sim_joint_RL_calf_sub = Subscriber{unitree_legged_msgs.msg.MotorState}("a1_gazebo/RL_calf_controller/state",
    #     sim_joint_RL_calf_callback, (fbk_state.motorState[12],), queue_size=1)

    # foot force sub
    sim_footforceFR_sub = Subscriber{geometry_msgs.msg.WrenchStamped}("visual/FR_foot_contact/the_force",
        sim_foot_force_FR_callback, (fbk_state,), queue_size=1)
    sim_footforceFL_sub = Subscriber{geometry_msgs.msg.WrenchStamped}("visual/FL_foot_contact/the_force",
        sim_foot_force_FL_callback, (fbk_state,), queue_size=1)
    sim_footforceRR_sub = Subscriber{geometry_msgs.msg.WrenchStamped}("visual/RR_foot_contact/the_force",
        sim_foot_force_RR_callback, (fbk_state,), queue_size=1)
    sim_footforceRL_sub = Subscriber{geometry_msgs.msg.WrenchStamped}("visual/RL_foot_contact/the_force",
        sim_foot_force_RL_callback, (fbk_state,), queue_size=1)

    # IMU sub
    sim_imu_sub = Subscriber{sensor_msgs.msg.Imu}("trunk_imu", sim_imu_callback, (fbk_state,), queue_size=1)

    """ send sim data data and pub"""
    simcmd_list = [unitree_legged_msgs.msg.MotorCmd() for i=1:12]
    sim_pub_list  = []
    pub = Publisher{unitree_legged_msgs.msg.MotorCmd}("a1_gazebo/FR_hip_controller/command", queue_size=1)
    push!(sim_pub_list,pub)
    pub = Publisher{unitree_legged_msgs.msg.MotorCmd}("a1_gazebo/FR_thigh_controller/command", queue_size=1)
    push!(sim_pub_list,pub)
    pub = Publisher{unitree_legged_msgs.msg.MotorCmd}("a1_gazebo/FR_calf_controller/command", queue_size=1)
    push!(sim_pub_list,pub)
    pub = Publisher{unitree_legged_msgs.msg.MotorCmd}("a1_gazebo/FL_hip_controller/command", queue_size=1)
    push!(sim_pub_list,pub)
    pub = Publisher{unitree_legged_msgs.msg.MotorCmd}("a1_gazebo/FL_thigh_controller/command", queue_size=1)
    push!(sim_pub_list,pub)
    pub = Publisher{unitree_legged_msgs.msg.MotorCmd}("a1_gazebo/FL_calf_controller/command", queue_size=1)
    push!(sim_pub_list,pub)
    pub = Publisher{unitree_legged_msgs.msg.MotorCmd}("a1_gazebo/RR_hip_controller/command", queue_size=1)
    push!(sim_pub_list,pub)
    pub = Publisher{unitree_legged_msgs.msg.MotorCmd}("a1_gazebo/RR_thigh_controller/command", queue_size=1)
    push!(sim_pub_list,pub)
    pub = Publisher{unitree_legged_msgs.msg.MotorCmd}("a1_gazebo/RR_calf_controller/command", queue_size=1)
    push!(sim_pub_list,pub)
    pub = Publisher{unitree_legged_msgs.msg.MotorCmd}("a1_gazebo/RL_hip_controller/command", queue_size=1)
    push!(sim_pub_list,pub)
    pub = Publisher{unitree_legged_msgs.msg.MotorCmd}("a1_gazebo/RL_thigh_controller/command", queue_size=1)
    push!(sim_pub_list,pub)
    pub = Publisher{unitree_legged_msgs.msg.MotorCmd}("a1_gazebo/RL_calf_controller/command", queue_size=1)
    push!(sim_pub_list,pub)

    """ start control loop """
    try
        loop(fbk_state, joy_data, base_state, simcmd_list, sim_pub_list)
    catch e
        if e isa InterruptException
           # cleanup
           println("control loop terminated by the user")

        #    rethrow(e)
        else
            println(e)
            rethrow(e)
            # println("some other error")
        end
    end
end

function sim_setCmdMotorTau(id::Int, tau::Float32, simcmd_list)
    simcmd_list[id+1].mode = 0x0A
    simcmd_list[id+1].tau = tau
end

function sim_SendCommand(simcmd_list, sim_pub_list)
    #ros send
    for i=1:12
        publish(sim_pub_list[i], simcmd_list[i])
    end
end


function loop(fbk_state, joy_data, base_state, simcmd_list, sim_pub_list)
    ctrl_hz = 250.0
    ctrl_dt = 1.0/ctrl_hz
    loop_rate = Rate(ctrl_hz)
    q_list = zeros(3,4)  # FR, FL, RR, RL
    dq_list = zeros(3,4)  # FR, FL, RR, RL

    rossleep(Rate(Duration(3)))
    """ get init robot position """
    ref_base_position = [base_state.position[1];
                         base_state.position[2];
                         base_state.position[3]];


    standup_down_z_vel = 0;
    control_state = 0      # 0 is standing
    # foot contact state
    #TODO: add a scheduling
    foot_contact = [1,1,1,1]

    mass = 12.5
    mg = @SVector[0,0,mass*9.81]
    normal_load = mg/4.0
    F_prev = vcat(normal_load,normal_load,normal_load,normal_load)
    # constants for QP
    miu = 0.5

    # Cx = d
    # x force
    # C physical meaning.
    Cb = @SMatrix [ 0    0    -1;
                    1    0  -miu;
                   -1    0  -miu;
                    0    1  -miu;
                    0   -1  -miu]
    # TODO: make this sparse?
    z3 = @SMatrix zeros(5,3)
    z33 = @SMatrix zeros(3,3)
    I3 = SMatrix{3,3}(1I)

    # Russ tedrake, friction cone


    C = [Cb z3 z3 z3;
        z3 Cb z3 z3;
        z3 z3 Cb z3;
        z3 z3 z3 Cb]
    C = SMatrix{20,12}(C)
    # the three parameters of the QP, different F components have different value
    alpha_x = alpha_y = alpha_z = 0.5
    beta_x = beta_y = beta_z = 0.05
    s_vec = [1,1,1,30,30,30]
    S = diagm(s_vec)
    S = SMatrix{6,6}(S)
    alpha_vec = [alpha_x,alpha_y,alpha_z]
    alpha_mtx = diagm(vcat(alpha_vec,alpha_vec,alpha_vec,alpha_vec))
    alpha_mtx = SMatrix{12,12}(alpha_mtx)
    beta_vec = [beta_x,beta_y,beta_z]
    beta_mtx = diagm(vcat(beta_vec,beta_vec,beta_vec,beta_vec))
    beta_mtx = SMatrix{12,12}(beta_mtx)

    """ variables used in the loop """
    # frame definition
    # e - world frame
    # b - body frame

    # R_eb, q_eb
    # pe = q_eb * pb
    # a mathematical intro to manipulation
    # modern robotics

    p_eb = zeros(3)
    v_eb = zeros(3)
    q_eb = UnitQuaternion(1.0,0.0,0.0,0.0)
    leg_ID = 0.0
    leg_tau = zeros(3)

    # debug pub
    debug_pub = Publisher{geometry_msgs.msg.Quaternion}("rosjl/debug/point1", queue_size=1)
    debug_point = geometry_msgs.msg.Quaternion()
    debug_pub2 = Publisher{geometry_msgs.msg.Point}("rosjl/debug/point2", queue_size=1)
    debug_point2 = geometry_msgs.msg.Point()

    # main ros loop
    while ! is_shutdown()

        """ get feedback """
        # get joint angles
        for i=1:4
            leg_ID = i-1 # for leg ID we all use C style
            for j=1:3
                q_list[j,i] = convert(Float64, fbk_state.motorState[leg_ID*3+j].q)
                dq_list[j,i] = convert(Float64, fbk_state.motorState[leg_ID*3+j].dq)
            end
        end
        # get current base states
        for i=1:3
            p_eb[i] = base_state.position[i]
            v_eb[i] = base_state.velocity[i]
        end
        q_eb = UnitQuaternion(base_state.orientation[1],
                              base_state.orientation[2],
                              base_state.orientation[3],
                              base_state.orientation[4])

        """ safety """
        if joy_data.buttons[5] == 1
            joy_data.buttons[5] = 0
            break
        end
        # curr_base_quat_tilt, curr_base_quat_torsion = quat_decompose_tilt_torsion(conj(curr_base_quat))
        # curr_base_yrp[1], curr_base_yrp[2], curr_base_yrp[3] = quat_to_euler(curr_base_quat_tilt)
        # println(curr_base_yrp/pi*180)
        # println("state_quat")
        # println([yaw/pi*180.0, roll/pi*180.0, pitch/pi*180.0])

        # show(stdout, "text/plain", curr_base_yrp/pi*180)
        # println(q_list[3,2])
        # println(fbk_state.imu.gyroscope)
        # println(fbk_state.motorState[1].q)
        # println(joy_data.axes)

        """ different control state """
        if control_state == 0
            # control state 0, the standup state
            # use joy stick axes 5 to control
            joy_input = convert(Float64, joy_data.axes[2])
            if (joy_input>0.2 || joy_input<-0.2)
                standup_down_z_vel = 0.2*joy_input
            else
                standup_down_z_vel = 0
            end
            # modify reference p and reference v
            # for i=1:4
            #     ref_p_list[3,i] += ctrl_dt*standup_down_z_vel
            #     if ref_p_list[3,i] > -0.01
            #         ref_p_list[3,i] = -0.01
            #     elseif ref_p_list[3,i] < -0.26
            #         ref_p_list[3,i] = -0.26
            #     end
            #     ref_v_list[3,i] = standup_down_z_vel
            # end

            ref_base_position[3] += ctrl_dt*standup_down_z_vel
            if ref_base_position[3] < 0.05
                ref_base_position[3] = 0.05
            elseif ref_base_position[3] > 0.3
                ref_base_position[3] = 0.3
            end

            tgt_yaw = joy_data.axes[1]*30/180*pi
            tgt_roll = joy_data.axes[4]*30/180*pi
            tgt_pitch = joy_data.axes[5]*30/180*pi

            q_tilt, q_torsion = quat_decompose_tilt_torsion(q_eb)
            q_tgt = q_torsion # TODO: get current yaw
            # q_tgt = UnitQuaternion(1.0,0.0,0.0,0.0) # TODO: get current yaw
            q_tgt = UnitQuaternion(RotZYX(tgt_yaw, tgt_roll, tgt_pitch)) # TODO: get current yaw
            q_err = q_tgt*conj(q_eb)

            # # curr_base_yaw, curr_base_roll, curr_base_pitch = quat_to_euler(q_eb)
            # debug_point.x = curr_base_yaw
            # debug_point.y = curr_base_roll
            # debug_point.z = curr_base_pitch
            # publish(debug_pub, debug_point)


            w_b = fbk_state.imu.gyroscope
            w_e = q_eb*w_b
            # println("current_w")        A1Robot.setCmdMotorTau(robot, leg_ID*3, Float32(tau[1]))
        A1Robot.setCmdMotorTau(robot, leg_ID*3+1, Float32(tau[2]))
        A1Robot.setCmdMotorTau(robot, leg_ID*3+2, Float32(tau[3]))

        A1Robot.SendCommand(robot)

            # println(current_w)

            Kp = diagm([120;220;80])
            Kd = diagm([5;25;10])
            Inertia = diagm([0.1;0.1;0.1])
            Ie = q_eb*Inertia*conj(q_eb)
            q_err_log = log(q_err)
            q_err_vec = @SVector[q_err_log.x,q_err_log.y,q_err_log.z]
            e_w = Kp*q_err_vec + Kd*(-w_e)
            tgt_wa = Ie*e_w
            # @printf("%6.4f \t %6.4f \t %6.4f \n", tgt_wa[1],tgt_wa[2],tgt_wa[3])
            debug_point2.x = tgt_wa[1]
            debug_point2.y = tgt_wa[2]
            debug_point2.z = tgt_wa[3]
            publish(debug_pub2, debug_point2)

            # println(ref_base_position)
            # println(p_eb)

            # construct QP
            Kp_a_val = 16.8
            Kd_a_val = 5.7
            Kp_a = diagm([100;100;20])
            Kd_a = diagm([45;45;10])
            ref_body_vel = [0;0;standup_down_z_vel]
            tgt_a = Kp_a*(ref_base_position-p_eb)+Kd_a*(ref_body_vel-v_eb)
            # println(tgt_a)
            ma = mass*tgt_a
            b = vcat(ma+mg,tgt_wa)

            pR_list = []
            # println(A1Robot.fbk_state.motorState)
            # show(stdout, "text/plain", q_list)
            for i=1:4
                leg_ID = i - 1

                p = A1Robot.fk(leg_ID, q_list[:,i])
                p = convert(SVector{3},p)
                # println("p")
                # println(p)
                pR = skew(q_eb*p) # this q convert distance to world franme
                push!(pR_list, pR)
            end

            pRmtx = []
            Imtx = []
            for i=1:4
                # if foot_contact[i] == 1
                    if isempty(pRmtx)
                        pRmtx = pR_list[i]
                        Imtx = I3
                    else
                        pRmtx = hcat(pRmtx, pR_list[i])
                        Imtx = hcat(Imtx,I3)
                    end
                # end
            end
            A = vcat(Imtx,pRmtx)  # always 6x12
            # println(A)

            # D confines the foot force
            D = sparse(I(12))
            ld = fill(-Inf,12)
            ud = fill(Inf,12)
            for i=1:4
                if foot_contact[i] == 0
                    ld[(i-1)*3:i*3] .= 0
                    ud[(i-1)*3:i*3] .= 0
                end
            end

            # QP
            # F'(A'*S*A+alpha+beta)F - (A'Sb+2 beta F_rev)'F
            # Cx<=0
            # Dx = 0

            # print(b)
            P = A'*S*A + alpha_mtx + beta_mtx
            # print(P)
            q = -(A'*S*b + 2*beta_mtx*F_prev)
            # print(q)
            model = OSQP.Model()
            OSQP.setup!(model, P=sparse(P), q=Vector(q), A=sparse([D; C]), l=[ld; fill(-Inf,20)], u=[ud; zeros(20)],
                eps_abs=1e-6, eps_rel=1e-6, verbose=false)
            res = OSQP.solve!(model)

            F = reshape(res.x,3,4)

            # show(stdout, "text/plain", F)
            # println("---")

            debug_point.x = F[3,1]
            debug_point.y = F[3,2]
            debug_point.z = F[3,3]
            debug_point.w = F[3,4]

            publish(debug_pub, debug_point)
            # debug_point2.x =0.0
            # debug_point2.y = F[3,4]
            # debug_point2.z = 0.0
            # publish(debug_pub2, debug_point2)

            # show(stdout, "text/plain", ref_p_list)
            F_prev = res.x

            for i=1:4
                leg_ID = i-1
                foot_F = conj(q_eb)*(-F[:,i]) # foot push into ground
                # foot_tau = zeros(3)
                # println(vcat(foot_tau,foot_F))
                # show(stdout, "text/plain", F)
                tau = A1Robot.stance_torque_ctrl(leg_ID, q_list[:,i], Array(foot_F))
                # println(tau)
                # println([fbk_state.motorState[leg_ID*3+1].tauEst;
                #          fbk_state.motorState[leg_ID*3+2].tauEst;
                        #  fbk_state.motorState[leg_ID*3+3].tauEst;])
                # tau = torque_ctrl(leg_ID, ref_p, ref_v, ref_a, q_list[:,i], dq_list[:,i], foot_F)
                # A1Robot.setCmdMotorTau(robot, leg_ID*3, Float32(tau[1]))
                # A1Robot.setCmdMotorTau(robot, leg_ID*3+1, Float32(tau[2]))
                # A1Robot.setCmdMotorTau(robot, leg_ID*3+2, Float32(tau[3]))
                sim_setCmdMotorTau(leg_ID*3, Float32(tau[1]),simcmd_list)
                sim_setCmdMotorTau(leg_ID*3+1, Float32(tau[2]),simcmd_list)
                sim_setCmdMotorTau(leg_ID*3+2, Float32(tau[3]),simcmd_list)
            end
        end

        sim_SendCommand(simcmd_list, sim_pub_list)
        rossleep(loop_rate)

    end
end


if ! isinteractive()
    main()
end
