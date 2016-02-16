classdef RoboticArmSingleLink < handle
    properties
        goal
        proximity
        pose_res
        pose_lim
        joints_lim
        joints_value
        joints_res
        nodes
        opened
        closed
        pose_path
        joint_path
        TF
        base_pose
        links_length
        number_of_links
        number_of_joints
        hed
        wsp %workspace plot
        half_link_width
    end
    
    methods
        function obj = RoboticArmSingleLink(links_length,joints_value,base_pose,joints_lim)
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
            
            obj.joints_res       = 20*pi/180;      % 5 deg
            obj.pose_res        = 0.05;           % meter
            obj.pose_lim        = [-1 1];
            obj.joints_lim      = joints_lim;
            obj.proximity       = 0.1;
            obj.hed             = [];
            obj.half_link_width = 0.025;
        end
        
        
        function path = FindPathAstar(obj,Xgoal,proximity,joints_res)
            % JOINT STATES A*
            if nargin==3
                obj.proximity   = proximity;
            elseif nargin==4
                obj.proximity   = proximity;
                obj.joints_res   = joints_res;      % 5 deg
            end
            
            %init closeset
            closedset           = [];
            L_closed            = 0;
            
            % generate initial node
            pose                = obj.FK(obj.joints_value);
            g                   = 0;
            h                   = obj.calcH(pose,Xgoal);
            f                   = g+h;
            nodelist            = struct('q',obj.joints_value,'pose',pose,'g',g,'h',h,'f',f,'parent',0,'id',1);
            L_nodelist          = 1;
            
            % for debug
            % plot(nodelist.pose(1),nodelist.pose(2),'bo')
            % hhh = plot(nodelist.pose(1),nodelist.pose(2),'ko')
            
            
            % init openset
            openset             = 1; % the first node in nodelist
            L_open              = 1;
            
            % start search
            while (L_open>0)
                % pick the best node in openset
                current = openset(1);
                index_in_openset = 1;
                for ii = 1:L_open
                    if (nodelist(current).f > nodelist(openset(ii)).f)
                        current = openset(ii);
                        index_in_openset = ii;
                    end
                end
                % for debbuging
%                 obj.plotArm(nodelist(current).q)
                
                % check if we've got to goal
                norm(nodelist(current).pose(1:2) - Xgoal(1:2))
                if norm(nodelist(current).pose(1:2) - Xgoal(1:2)) <= obj.proximity
                    % goal found
                    disp('path founded');
                    % generate solution
                    obj.joint_path   = obj.reconstruct_path(nodelist,current);
                    obj.nodes       = nodelist;
                    obj.opened      = openset;
                    obj.closed      = closedset;
                    path            = obj.joint_path;
                    return;
                end
                
                % remove current from openset
                if L_open == 1
                    openset     = [];
                    L_open      = 0;
                elseif index_in_openset==1
                    openset     = openset(index_in_openset+1:end);
                    L_open      = L_open - 1;
                elseif index_in_openset==L_open
                    openset     = openset(1:index_in_openset-1);
                    L_open      = L_open - 1;
                else
                    openset     = [openset(1:index_in_openset-1) ; openset(index_in_openset+1:end)];
                    L_open      = L_open - 1;
                end
                
                % add current to closed set
                closedset   = [closedset ; current];
                L_closed    = L_closed + 1;
                
                
                for q1 = -1:1
                    if (q1==0)
                        continue
                    end
                    
                    neighbor.q      = nodelist(current).q + obj.joints_res*[q1];
                    neighbor.pose   = obj.FK(neighbor.q);%nodelist(current).pose + obj.pose_res*[q1 q2];
                    
                    % for debugging
                    % plot(neighbor.pose(1),neighbor.pose(2),'ro')
                    
                    % check kinematic feasibility
                    if any(neighbor.q<obj.joints_lim(1) | neighbor.q>obj.joints_lim(2))
                        continue;
                    end
                    
                    % check if neighbor is in closedset
                    founded = 0;
                    for nn = 1:L_closed
                        if all(abs(nodelist(closedset(nn)).q - neighbor.q) < 0.00001)
                            founded = 1;
                            break;
                        end
                    end
                    if founded
                        continue;
                    end
                    
                    % neighbor.pose = obj.FK(neighbor.q);
                    tentative_g_score = nodelist(current).g + norm(nodelist(current).pose - neighbor.pose);
                    
                    % check if neighbor is in openset
                    founded = 0;
                    for nn = 1:L_open
                        if all(nodelist(openset(nn)).q==neighbor.q)
                            founded = nn;
                            break;
                        end
                    end
                    if founded==0
                        % add this node to nodellist
                        neighbor.g = Inf;
                        neighbor.h = obj.calcH(neighbor.pose,Xgoal);
                        neighbor.f = Inf;
                        neighbor.parent = [];
                        neighbor.id = L_nodelist + 1;
                        
                        nodelist = [nodelist ; neighbor];
                        L_nodelist = L_nodelist + 1;
                        
                        % for debugging
                        % plot(neighbor.pose(1),neighbor.pose(2),'ro')
                        
                        % add this node to openset
                        openset = [openset ; neighbor.id];
                        L_open = L_open + 1;
                        
                        nodelist(neighbor.id).parent    = current;
                        nodelist(neighbor.id).g         = tentative_g_score;
                        nodelist(neighbor.id).f         = tentative_g_score + nodelist(neighbor.id).h;
                        
                        neighbor = [];
                        
                    elseif tentative_g_score <= nodelist(openset(nn)).g
                        % this is a better path
                        nodelist(openset(nn)).parent    = current;
                        nodelist(openset(nn)).g         = tentative_g_score;
                        nodelist(openset(nn)).f         = tentative_g_score + nodelist(openset(nn)).h;
                    end
                end
            end
        end
        
        
        function path = reconstruct_path(obj,nodelist,current)
            ind = current;
            path  = [];
            while (ind~=0)
                path  = [path  ; nodelist(ind).q(:)];
                ind = nodelist(ind).parent;
            end
            path = flipud(path);
        end
        
        function H = calcH(obj,pos,goal)
            H = norm(pos-goal);
        end
        
        function ver = calcArmVertex(obj,q)
            if nargin==1
                q = obj.joints_value;
            end
            l1 = obj.links_length(1);
            l2 = obj.links_length(2);
            l3 = obj.links_length(3);
            x0 = obj.base_pose.x;
            y0 = obj.base_pose.y;
            psi0 = obj.base_pose.yaw;
            q1 = q(1);
            q2 = q(2);
            q3 = q(3);
            
            t2 = psi0+q1;
            t3 = cos(t2);
            t4 = l1.*t3;
            t5 = sin(t2);
            t6 = l1.*t5;
            t7 = psi0+q1+q2;
            t8 = cos(t7);
            t9 = l2.*t8;
            t10 = psi0+q1+q2+q3;
            t11 = sin(t7);
            t12 = l2.*t11;
            ver = reshape([x0,t4+x0,t4+t9+x0,t4+t9+x0+l3.*cos(t10),y0,t6+y0,t6+t12+y0,t6+t12+y0+l3.*sin(t10)],[4, 2]);
        end
        
        function L = calcLinksVertexs(obj,q)
            if nargin==1
                q = obj.joints_value;
            end
            
            linkID = 1;
            vertex = [ [-obj.half_link_width;obj.half_link_width;1] , ...
                [obj.links_length(linkID) + obj.half_link_width;obj.half_link_width;1] , ...
                [obj.links_length(linkID) + obj.half_link_width;-obj.half_link_width;1] , ...
                [-obj.half_link_width;-obj.half_link_width;1] ];
            L = obj.base_pose.tf*obj.TF(0,0,q(linkID))*vertex;
            L = L(1:2,:);
        end
               
        function ver = calcArmVertexs(obj,q)
            if nargin==1
                q = obj.joints_value;
            end
            
            p0 = [obj.base_pose.x ; obj.base_pose.y];
            
            linkID = 1;

            m1 = obj.base_pose.tf*obj.TF(obj.links_length(linkID),0,q(linkID)) ;
            p1 = m1(1:2,3);
            
            ver = [p0 , p1]';

        end
        
        function X = FK(obj,q)
            if nargin==1
                q = obj.joints_value;
            end
            X = [obj.base_pose.x + obj.links_length(1)*cos(obj.base_pose.yaw + q(1))
                obj.base_pose.y + obj.links_length(1)*sin(obj.base_pose.yaw + q(1))
                obj.base_pose.yaw + q(1)];
        end
        
        % plotting
        
        function plotJointPath(obj)
            for ii = length(obj.joint_path):-1:1
                obj.plotArm(obj.nodes(obj.joint_path(ii)).q);
                pause(0.3);
            end
        end
        
        function hed = plotArm(obj,q)
            if ~isempty(obj.hed)
                delete(obj.hed(:))
            end
            if nargin==1
                q = obj.joints_value;
            end
            tf_base_pose = obj.TF(obj.base_pose.x,obj.base_pose.y,0)*obj.TF(0,0,obj.base_pose.yaw);
            
            tf = tf_base_pose*...
                obj.TF(0,0,q(1));
            link_cart = tf * [0 , obj.links_length(1)
                0 , 0
                1 , 1];
            gripper_cart = tf * obj.TF(obj.links_length(1),0,0)*...
                [0.05  , 0     , 0       , 0.05 
                 0.025 , 0.025 , -0.025  , -0.025
                 1     , 1     , 1       ,  1    ];
            
            hed(1) = plot(obj.base_pose.x,obj.base_pose.y,'s',...
                'MarkerEdgeColor',[0 0.447058826684952 0.74117648601532],...'Marker','o',...
                'LineWidth',3,...
                'Color',[1 0 0]);
%             hed(2) = plot(link_cart(1,:),link_cart(2,:),'MarkerFaceColor',[0 0 1],...
%                 'MarkerEdgeColor',[0 0.447058826684952 0.74117648601532],...'Marker','o',...
%                 'LineWidth',3,...
%                 'Color',[1 0 0]);
%             hed(3) = plot(gripper_cart(1,:),gripper_cart(2,:),'MarkerFaceColor',[0 0 1],...
%                 'MarkerEdgeColor',[0 0.447058826684952 0.74117648601532],...'Marker','o',...
%                 'LineWidth',3,...
%                 'Color',[0 0 1]);
            
            
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
