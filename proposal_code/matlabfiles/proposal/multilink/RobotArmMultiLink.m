classdef RobotArmMultiLink < handle
    properties
        joints_lim
        joints_value
        branching_matrix
        branching_factor
        TF
        base_pose
        links_length
        number_of_links
        number_of_joints
        hed
        wsp %workspace plot
        half_link_width
        path
    end
    
    methods
        function obj = RobotArmMultiLink(links_length,joints_value,base_pose,joints_limits)
            % initialization
            obj.joints_value     = joints_value;
            obj.links_length     = links_length;
            obj.number_of_links  = length(links_length);
            obj.number_of_joints = length(links_length);
            
            obj.TF = @(x,y,psi) [ cos(psi), -sin(psi), x*cos(psi) - y*sin(psi)
                sin(psi),  cos(psi), y*cos(psi) + x*sin(psi)
                0,         0,                       1];
            
            obj.base_pose       = base_pose;
            obj.base_pose.tf    = obj.TF(base_pose.x,base_pose.y,0)*obj.TF(0,0,base_pose.yaw);
            
            obj.joints_lim      = joints_limits;
            obj.hed             = [];
            obj.half_link_width = 0.025;
            
            obj.update_branching_matrix();
        end
        
        function pose = FK(obj,joints_value)
            % Calc the Forward Kinematics (global position) of the gripper
            % pose = [x ; y ; yaw]
            
            if nargin==1
                q = obj.joints_value;
            else
                q = joints_value
            end

            mat = obj.base_pose.tf;
            for ii = 1:obj.number_of_joints
               mat = mat*obj.TF(obj.links_length(ii),0,q(ii));
            end
            
            pose = [mat(1:2,3) ; atan2(mat(2,1),mat(1,1))];
        end
        
        function  update_branching_matrix(obj)

            branching_factor = 3^obj.number_of_joints;
            branching_matrix = zeros(branching_factor,obj.number_of_joints);
            
            for jj = 1:obj.number_of_joints
                vect = [ -1*ones(3^(jj-1),1) 
                          0*ones(3^(jj-1),1) 
                          1*ones(3^(jj-1),1) ];
                
                branching_matrix(:,jj) = repmat(vect,[3^(obj.number_of_joints-jj),1]);
            end
            
            obj.branching_matrix = branching_matrix;
            obj.branching_factor = branching_factor;
            
            
        end
        
        function [ver,mat] = calcJointsPosition(obj,q) 
            % Calc the global position of each joint
            % this includes the tip of the arm
            % ver = nx2 matrix
            % mat = transform matrix to the tip of the arm
            if nargin==1
                q = obj.joints_value;
            end

            ver = [obj.base_pose.x obj.base_pose.y];
            mat = obj.base_pose.tf;
            
            for ii = 1:obj.number_of_joints
               mat = mat*obj.TF(obj.links_length(ii),0,q(ii));
               ver(end+1,:) = mat(1:2,3)';
            end
        end
        
        function allvertex = calcLinksVertexs(obj,q)
            % returns a 2x4xnum_of_links matrix. Each z fits to a link.
            if nargin==1
                q = obj.joints_value;
            end
            allvertex = [];
            mat = obj.base_pose.tf;
            
            for ii = 1:obj.number_of_joints
               mat = mat*obj.TF(0,0,q(ii));
               allvertex(:,:,ii) = mat*[ [-obj.half_link_width;obj.half_link_width;1] , ...
                   [obj.links_length(ii) + obj.half_link_width;obj.half_link_width;1] , ...
                   [obj.links_length(ii) + obj.half_link_width;-obj.half_link_width;1] , ...
                   [-obj.half_link_width;-obj.half_link_width;1] ];
               mat = mat*obj.TF(obj.links_length(ii),0,0);
            end
            
            allvertex = allvertex(1:2,:,:);
            
        end
        
        % plotting
        
        function hed = plotArm(obj,q)
            if ~isempty(obj.hed)
                delete(obj.hed(:))
            end
            if nargin==1
                q = obj.joints_value;
            end
            
            [link_cart,tip_tf] = obj.calcJointsPosition(q);
            
            gripper_cart = tip_tf*...
                [0.05  , 0     , 0       , 0.05 
                 0.025 , 0.025 , -0.025  , -0.025
                 1     , 1     , 1       ,  1    ];
            
            hed(1) = plot(obj.base_pose.x,obj.base_pose.y,'s',...
                'MarkerEdgeColor',[0 0.447058826684952 0.74117648601532],...'Marker','o',...
                'LineWidth',3,...
                'Color',[1 0 0]);

            hed(2) = plot(link_cart(:,1),link_cart(:,2),'MarkerFaceColor',[0 0 1],...
                'MarkerEdgeColor',[0 0.447058826684952 0.74117648601532],...'Marker','o',...
                'LineWidth',3,...
                'Color',[1 0 0]);
            hed(3) = plot(gripper_cart(1,:),gripper_cart(2,:),'MarkerFaceColor',[0 0 1],...
                'MarkerEdgeColor',[0 0.447058826684952 0.74117648601532],...'Marker','o',...
                'LineWidth',3,...
                'Color',[0 0 1]);
            
            
            obj.hed = hed;
        end
        
        function plotWorkSpace(obj)
            t = 0:0.02:2*pi;
            x = obj.base_pose.x + sum(obj.links_length)*cos(t);
            y = obj.base_pose.y + sum(obj.links_length)*sin(t);
            
            obj.wsp = patch (x,y,ones(size(x)),'FaceAlpha',.3);
        end
        
    end
end
