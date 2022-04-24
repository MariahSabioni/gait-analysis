classdef id
    methods (Static)
        %% Read marker trajectory and ground reaction data
        % data files should be in the same folder as the .m file
        function [data_trc, data_grf_s]=load_file(motion)
            name_motion = strcat(motion,'.txt');
            name_grf = strcat(motion,'FP','.txt');
            data_trc = readtable([name_motion]);
            data_grf = readtable([name_grf]);
            data_grf_s = downsample(data_grf,10);
        end
        %% Set initial and final frames
        function [frames_r, frames_l] = get_frames(motion)
            % normal
            norm_l_frame = [295 393];
            norm_r_frame = [245 344];
            % highjump
            highjump_r_frame = [563 598];
            % lowjump
            lowjump_r_frame = [149 278];
            if isequal(motion, 'NormWalk')
                frames_r = norm_r_frame;
                frames_l = norm_l_frame;
            else
                frames_r = highjump_r_frame;
                frames_l = lowjump_r_frame;
            end
        end
        %% Assign the points to variables in MATLAB
        function [LTOE,LANKLE,LKNEE,LHIP,RTOE,RANKLE,RKNEE,RHIP,PELO,PELP,TRXO,TRXP] = get_points(conversion, data_trc)
            toMeters=conversion; 
            LTOE = [data_trc.LTOO_Y*toMeters, data_trc.LTOO_Z*toMeters];   
            LANKLE = [data_trc.LAJC_Y*toMeters, data_trc.LAJC_Z*toMeters];   
            LKNEE = [data_trc.LKJC_Y*toMeters, data_trc.LKJC_Z*toMeters];   
            LHIP = [data_trc.LHJC_Y*toMeters, data_trc.LHJC_Z*toMeters];        
            
            % Find coordinations of RTOE, RANKLE, RKNEE, RHIP
            RTOE = [data_trc.RTOO_Y*toMeters, data_trc.RTOO_Z*toMeters];   
            RANKLE = [data_trc.RAJC_Y*toMeters, data_trc.RAJC_Z*toMeters];   
            RKNEE= [data_trc.RKJC_Y*toMeters, data_trc.RKJC_Z*toMeters];   
            RHIP = [data_trc.RHJC_Y*toMeters, data_trc.RHJC_Z*toMeters];   
            
            % Find coordinations of PELO, PELP, TRXO, TRXP
            PELO = [data_trc.PELO_Y*toMeters, data_trc.PELO_Z*toMeters]; 
            PELP  = [data_trc.PELP_Y*toMeters, data_trc.PELP_Z*toMeters];   
            TRXO = [data_trc.TRXO_Y*toMeters, data_trc.TRXO_Z*toMeters];   
            TRXP = [data_trc.TRXP_Y*toMeters, data_trc.TRXP_Z*toMeters];  
        end
        %% Assign the grd reaction forces to variables in MATLAB
        function [FP1_force, FP1_COP, FP2_force, FP2_COP] = get_grd(conversion, motion, data_grf_s)
            toMeters=conversion; 
            if isequal(motion,'NormWalk')
                FP1_force = [data_grf_s.FP1_Force_Y, data_grf_s.FP1_Force_Z];
                FP1_COP = [data_grf_s.FP1_COP_Y*toMeters, data_grf_s.FP1_COP_Z*toMeters];
                FP2_force = [data_grf_s.FP2_Force_Y, data_grf_s.FP2_Force_Z];
                FP2_COP = [data_grf_s.FP2_COP_Y*toMeters, data_grf_s.FP2_COP_Z*toMeters];
            else
                FP1_force = [data_grf_s.FP4_Force_Y, data_grf_s.FP4_Force_Z];
                FP1_COP = [data_grf_s.FP4_COP_Y*toMeters, data_grf_s.FP4_COP_Z*toMeters];
                FP2_force = [data_grf_s.FP2_Force_Y, data_grf_s.FP2_Force_Z]; 
                FP2_COP = [data_grf_s.FP2_COP_Y*toMeters, data_grf_s.FP2_COP_Z*toMeters]; 
            end
        end
        %% Trim data
        function [trim_array] = trim(array, frames)
            trim_array = array(frames(1,1):frames(1,2),:);
        end
        %% Compute absolute angles of a segment
        function [segment_ang] = get_seg_angle(distal, proximal)
            segment_vector = distal - proximal;
            segment_ang = id.get_segment_angles(segment_vector, 'y');
        end
        %% Compute absolute angle of a vector angles
        function [v_angles] = get_segment_angles(vector, axis)
            for i=1:length(vector)
                if axis == 'y' %angle relative to y axis
                    v_angles(i,:) = (atan2d(vector(i,1),vector(i,2)));
                elseif axis == 'x'
                    v_angles(i,:) = (atan2d(vector(i,2),vector(i,1)));
                end
            end
        end
        %% Compute relative angles of a joint
        function [joint_angle] = get_joint_angle(distal, proximal, reference)
            joint_angle = distal - proximal + reference;
        end
        %% Calculate dteta/dt and d2teta/dt2
        function [vel, acc] = get_derivative(angle, timestep)
            for i=1:size(angle,2)
                vel(:,i) = gradient(angle(:,i), timestep);
                acc(:,i) = gradient(vel(:,i), timestep);
            end
        end
        %% Calculate segment length by averaging left and right 
        function [seg_len] = get_seg_len(rdistal, rproximal, ldistal, lproximal, frames_r, frames_l)
            seg_len_r = norm([rdistal(frames_r(1,1),1)-rproximal(frames_r(1,1),1) rdistal(frames_r(1,2),2)-rproximal(frames_r(1,1),2)]);
            seg_len_l = norm([ldistal(frames_l(1,1),1)-lproximal(frames_l(1,1),1) ldistal(frames_l(1,2),2)-lproximal(frames_l(1,1),2)]);
            seg_len = (seg_len_r+seg_len_l)/2;
        end
        %% Calculate the position of the center of mass in each frame
        function [seg_com] = get_com (distal, proximal, com)
            y=1; z=2;
            seg_com = (distal-proximal)*com+proximal;
        end
        %% Calculate acceleration of COM of a segment
        function [acc] = get_seg_acc(pos, timestep)
            %calculate acceleration
            vel(:,1) = gradient(pos(:,1),timestep);
            vel(:,2) =  gradient(pos(:,2),timestep);
            acc(:,1) = gradient(vel(:,1),timestep);
            acc(:,2) =  gradient(vel(:,2),timestep);
        end
        %% Calculate joint moments
        function [joint_moment, joint_force] = get_joint_moment(mass, Ig, seg_ang_acc, seg_acc, seg_com, M_1, contact_force_1, contact_point_1, contact_point_2)
            contact_force_1 = [zeros(size(contact_force_1,1),1) contact_force_1];
            contact_point_1 = [zeros(size(contact_point_1,1),1) contact_point_1];
            g = [zeros(size(seg_acc,1),1) zeros(size(seg_acc,1),1) (-9.81)*ones(size(seg_acc,1),1)];
            seg_com = [zeros(size(seg_com,1),1) seg_com];
            seg_acc = [zeros(size(seg_acc,1),1) seg_acc];
            contact_point_2 = [zeros(size(contact_point_2,1),1) contact_point_2];
            r1 = (contact_point_1 - seg_com);
            r2 = (contact_point_2 - seg_com);
            contact_force_2 = mass*(seg_acc - g) - contact_force_1;
            joint_moment = (Ig*seg_ang_acc - cross(r1,contact_force_1) - cross(r2,contact_force_2) - M_1);
            joint_moment = joint_moment(:,1);
            joint_force = contact_force_2(:, 2:3);
        end
        %% Plot graph with right and left sides
        function plot_2_var(var_r, var_l, title_name, pos_ref, neg_ref, units)
            nframes_r = size(var_r,1);
            nframes_l = size(var_l,1);
            gait_percentage_r = (linspace(0,nframes_r,nframes_r)/100)';
            gait_percentage_l = (linspace(0,nframes_l,nframes_l)/100)';
            figure
            plot(gait_percentage_r,var_r, 'LineWidth',1,'Color','g')
            hold on
            plot(gait_percentage_l, var_l, 'LineWidth',1,'Color','r');
            title(title_name,'FontSize',15 )
            legend('right cycle','left cycle')
            xlabel('Gait Percentage [%]','FontSize',14);
            ylabel(strcat(units," | (-) ", neg_ref," & (+) ", pos_ref), 'FontSize',14);
            grid on
        end
        %% Calculate joint powers
        function [joint_power]=get_power(moment, ang_vel)
            joint_power = moment.*ang_vel;
        end
    end
end


