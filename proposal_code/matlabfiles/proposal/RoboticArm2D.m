classdef RoboticArm2D < handle
    properties
        goal
        proximity
        angle_res
        pose_res
        pose_lim
        joints_lim
        joints_value
        nodes
        opened
        closed
        pose_path
        joint_path
        TF
        base_pose
        links_length
        hed
        half_link_width
    end
    
    methods
        function obj = RoboticArm2D(links_length,joints,base_pose,joints_lim)
            % initialization
            obj.joints_value = joints;
            obj.links_length = links_length;
            
            obj.base_pose = base_pose;
            
            obj.TF = @(x,y,psi) [ cos(psi), -sin(psi), x*cos(psi) - y*sin(psi)
                sin(psi),  cos(psi), y*cos(psi) + x*sin(psi)
                0,         0,                       1];
            
            obj.angle_res       = 20*pi/180;      % 5 deg
            obj.pose_res        = 0.05;           % meter
            obj.pose_lim        = [-1 1];
            obj.joints_lim      = joints_lim;
            obj.proximity       = 0.1;
            obj.hed             = [];
            obj.half_link_width = 0.05;
        end
        
        
        function path = FindPathAstarPose(obj,Xgoal,proximity,pose_res,angle_res)
            obj.proximity       = proximity;
            obj.pose_res       = pose_res;      % 5 deg
            
            %init closeset
            closedset           = [];
            L_closed            = 0;
            
            % generate initial node
            q                   = obj.joints_value;
            pose                = obj.FK(q);
            g                   = 0;
            h                   = obj.calcH(pose,Xgoal);
            f                   = g+h;
            nodelist            = struct('pose',pose,'q',q,'g',g,'h',h,'f',f,'parent',0,'id',1);
            L_nodelist          = 1;
            
            % for debug
            %             plot(nodelist.pose(1),nodelist.pose(2),'bo')
            %             hhh = plot(nodelist.pose(1),nodelist.pose(2),'ko')
            
            % init openset
            openset             = 1; % the first node in nodelist
            L_open              = 1;
            
            % start search
            while (L_open>0)
                % pick the best node in openset
                current = openset(1);
                for ii = 1:L_open
                    if (nodelist(current).f > nodelist(openset(ii)).f)
                        current = openset(ii);
                    end
                end
                % for debbuging
                %                 plot(nodelist(current).pose(1),nodelist(current).pose(2),'bo')
                %                 delete(hhh);
                %                 hhh = plot(nodelist(current).pose(1),nodelist(current).pose(2),'ko')
                
                % check if we've got to goal
                norm(nodelist(current).pose - Xgoal)
                if norm(nodelist(current).pose - Xgoal) <= obj.proximity
                    % goal found
                    disp('path founded');
                    % generate solution
                    obj.pose_path   = obj.reconstruct_path(nodelist,current);
                    obj.nodes       = nodelist;
                    obj.opened      = openset;
                    obj.closed      = closedset;
                    path            = obj.pose_path;
                    return;
                end
                % remove current from openset
                if L_open == 1
                    openset     = [];
                    L_open      = 0;
                else
                    openset     = openset(2:end);
                    L_open      = L_open - 1;
                end
                
                % add current to closed set
                closedset   = [closedset ; current];
                L_closed    = L_closed + 1;
                
                
                for xx = -1:1
                    for yy = -1:1
                        for yaya = -1:1
                            if ((xx==0) && (yy==0) && (yaya==0) )
                                continue
                            end
                            
                            neighbor.pose   = nodelist(current).pose + [pose_res*[xx yy] angle_res*yaya];
                            neighbor.q      = obj.IK(neighbor.pose);
                            
                            % for debugging
                            % plot(neighbor.pose(1),neighbor.pose(2),'ro')
                            
                            % check kinematic feasibility
                            if any(imag(neighbor.q)~=0)
                                continue;
                            end
                            
                            % check if neighbor is in closedset
                            founded = 0;
                            for nn = 1:L_closed
                                if all(nodelist(closedset(nn)).q==neighbor.q)
                                    founded = 1;
                                    break;
                                end
                            end
                            if founded
                                continue;
                            end
                            
                            %                             neighbor.pose = obj.FK(neighbor.q);
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
                            elseif tentative_g_score >= nodelist(openset(nn)).g
                                % this is not a better path
                                continue;
                            end
                            
                            nodelist(neighbor.id).parent    = current;
                            nodelist(neighbor.id).g         = tentative_g_score;
                            nodelist(neighbor.id).f         = tentative_g_score + nodelist(neighbor.id).h;
                            %                         end
                        end
                    end
                end
            end
        end
        
        function path = FindPathAstarJoints(obj,Xgoal,proximity,angle_res)
            obj.proximity       = proximity;
            obj.angle_res       = angle_res;      % 5 deg
            
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
            %             plot(nodelist.pose(1),nodelist.pose(2),'bo')
            %             hhh = plot(nodelist.pose(1),nodelist.pose(2),'ko')
            
            
            % init openset
            openset             = 1; % the first node in nodelist
            L_open              = 1;
            
            % start search
            while (L_open>0)
                % pick the best node in openset
                current = openset(1);
                for ii = 1:L_open
                    if (nodelist(current).f > nodelist(openset(ii)).f)
                        current = openset(ii);
                    end
                end
                % for debbuging
                %                 plot(nodelist(current).pose(1),nodelist(current).pose(2),'bo')
                %                 delete(hhh);
                %                 hhh = plot(nodelist(current).pose(1),nodelist(current).pose(2),'ko')
                
                % check if we've got to goal
                norm(nodelist(current).pose - Xgoal)
                if norm(nodelist(current).pose - Xgoal) <= obj.proximity
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
                else
                    openset     = openset(2:end);
                    L_open      = L_open - 1;
                end
                
                % add current to closed set
                closedset   = [closedset ; current];
                L_closed    = L_closed + 1;
                
                
                for q1 = -1:1
                    for q2 = -1:1
                        for q3 = -1:1
                            if ((q1==0) && (q2==0) && (q3==0) )
                                continue
                            end
                            
                            neighbor.q      = nodelist(current).q + obj.angle_res*[q1 q2 q3];
                            neighbor.pose   = obj.FK(neighbor.q);%nodelist(current).pose + obj.pose_res*[q1 q2];
                            
                            % for debugging
                            % plot(neighbor.pose(1),neighbor.pose(2),'ro')
                            
                            % check kinematic feasibility
                            %                             if any(neighbor.q<obj.angle_lim(1) | neighbor.q>obj.angle_lim(2))
                            %                                 continue;
                            %                             end
                            
                            % check if neighbor is in closedset
                            founded = 0;
                            for nn = 1:L_closed
                                if all(nodelist(closedset(nn)).q==neighbor.q)
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
                                if all(nodelist(openset(nn)).pose==neighbor.pose)
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
                            elseif tentative_g_score >= nodelist(openset(nn)).g
                                % this is not a better path
                                continue;
                            end
                            
                            nodelist(neighbor.id).parent    = current;
                            nodelist(neighbor.id).g         = tentative_g_score;
                            nodelist(neighbor.id).f         = tentative_g_score + nodelist(neighbor.id).h;
                        end
                    end
                end
            end
        end
        
        
        function path = reconstruct_path(obj,nodelist,current)
            ind = current;
            path = [];
            while (ind~=0)
                path = [path ; ind];
                ind = nodelist(ind).parent;
            end
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
        
        function [L1,L2,L3] = calcLinksVertex(obj,q)
            if nargin==1
                q = obj.joints_value;
            end
            half_link_width = obj.half_link_width;
            l1 = obj.links_length(1);
            l2 = obj.links_length(2);
            l3 = obj.links_length(3);
            x0 = obj.base_pose.x;
            y0 = obj.base_pose.y;
            psi0 = obj.base_pose.yaw;
            q1 = q(1);
            q2 = q(2);
            q3 = q(3);
            
            t2 = cos(psi0);
            t3 = cos(q1);
            t4 = sin(psi0);
            t5 = sin(q1);
            t6 = t2.*t3;
            t12 = t4.*t5;
            t7 = t6-t12;
            t8 = t2.*t5;
            t9 = t3.*t4;
            t10 = t8+t9;
            t11 = half_link_width+l1;
            t13 = t7.*t11;
            t14 = half_link_width.*t10;
            t15 = half_link_width.*t7;
            t16 = t10.*t11;
            L1 = reshape([x0-half_link_width.*t7-half_link_width.*t10,-t14+t15+y0,t13+x0-half_link_width.*t10,t15+t16+y0,t13+t14+x0,-t15+t16+y0,t14+x0-half_link_width.*t7,-t14-t15+y0],[2, 4]);
            if nargout > 1
                t17 = cos(q2);
                t18 = sin(q2);
                t19 = t7.*t17;
                t26 = t10.*t18;
                t20 = t19-t26;
                t21 = t10.*t17;
                t22 = t7.*t18;
                t23 = t21+t22;
                t24 = l1.*t2.*t3;
                t25 = half_link_width+l2;
                t27 = t20.*t25;
                t28 = half_link_width.*t23;
                t29 = half_link_width.*t20;
                t30 = l1.*t2.*t5;
                t31 = l1.*t3.*t4;
                t32 = t23.*t25;
                L2 = reshape([t24+x0-half_link_width.*t20-half_link_width.*t23-l1.*t4.*t5,-t28+t29+t30+t31+y0,t24+t27+x0-half_link_width.*t23-l1.*t4.*t5,t29+t30+t31+t32+y0,t24+t27+t28+x0-l1.*t4.*t5,-t29+t30+t31+t32+y0,t24+t28+x0-half_link_width.*t20-l1.*t4.*t5,-t28-t29+t30+t31+y0],[2, 4]);
            end
            if nargout > 2
                t33 = cos(q3);
                t34 = sin(q3);
                t35 = t20.*t33;
                t42 = t23.*t34;
                t36 = t35-t42;
                t37 = t23.*t33;
                t38 = t20.*t34;
                t39 = t37+t38;
                t40 = l2.*t7.*t17;
                t41 = half_link_width+l3;
                t43 = t36.*t41;
                t44 = half_link_width.*t39;
                t45 = half_link_width.*t36;
                t46 = l2.*t10.*t17;
                t47 = l2.*t7.*t18;
                t48 = t39.*t41;
                L3 = reshape([t24+t40+x0-half_link_width.*t36-half_link_width.*t39-l1.*t4.*t5-l2.*t10.*t18,t30+t31-t44+t45+t46+t47+y0,t24+t40+t43+x0-half_link_width.*t39-l1.*t4.*t5-l2.*t10.*t18,t30+t31+t45+t46+t47+t48+y0,t24+t40+t43+t44+x0-l1.*t4.*t5-l2.*t10.*t18,t30+t31-t45+t46+t47+t48+y0,t24+t40+t44+x0-half_link_width.*t36-l1.*t4.*t5-l2.*t10.*t18,t30+t31-t44-t45+t46+t47+y0],[2, 4]);
            end
        end
        
        function X = FK(obj,q)
            if nargin==1
                q = obj.joints_value;
            end
            X = [obj.base_pose.x + obj.links_length(3)*cos(obj.base_pose.yaw + q(1) + q(2) + q(3)) + obj.links_length(1)*cos(obj.base_pose.yaw + q(1)) + obj.links_length(2)*cos(obj.base_pose.yaw + q(1) + q(2)) , ...
                obj.base_pose.y + obj.links_length(3)*sin(obj.base_pose.yaw + q(1) + q(2) + q(3)) + obj.links_length(1)*sin(obj.base_pose.yaw + q(1)) + obj.links_length(2)*sin(obj.base_pose.yaw + q(1) + q(2)) , ...
                obj.base_pose.yaw + q(1) + q(2) + q(3)                                                                                                                                                           ];
        end
        
        function [Q1,Q2] = IK(obj,pose)
            l1 = obj.links_length(1);
            l2 = obj.links_length(2);
            l3 = obj.links_length(3);
            x0 = obj.base_pose.x;
            y0 = obj.base_pose.y;
            psi0 = obj.base_pose.yaw;
            
            x = pose(1);
            y = pose(2);
            yaw = pose(3);
            
            t2 = cos(yaw);
            t6 = l3.*t2;
            t7 = t6-x+x0;
            t3 = abs(t7);
            t4 = sin(yaw);
            t9 = l3.*t4;
            t10 = t9-y+y0;
            t5 = abs(t10);
            t8 = t3.^2;
            t11 = t5.^2;
            t12 = 1.0./l1;
            t13 = l1.^2;
            t14 = l2.^2;
            t15 = y.*1i;
            t16 = y0.*-1i;
            t17 = l3.*t4.*-1i;
            t18 = t8+t11;
            t19 = 1.0./sqrt(t18);
            t20 = t8+t11+t13-t14;
            t21 = t12.*t19.*t20.*(1.0./2.0);
            t22 = acos(t21);
            t23 = 1.0./l2;
            t24 = t8+t11-t13-t14;
            t25 = t12.*t23.*t24.*(1.0./2.0);
            t26 = acos(t25);
            t27 = -t6+t15+t16+t17+x-x0;
            t28 = angle(t27);
            Q1 = [-psi0+t22+angle(t15+t16+t17+x-x0-l3.*t2),-t26,-t22+t26-t28+yaw];
            if nargout > 1
                Q2 = [-psi0-t22+t28,t26,t22-t26-t28+yaw];
            end
        end
        
        % plotting
        
        function plotJointPath(obj)
            for ii = length(obj.joint_path):-1:1
                obj.plotArm(obj.nodes(obj.joint_path(ii)).q);
                pause(0.3);
            end
        end
        
        function plotPosePath(obj)
            for ii = length(obj.pose_path):-1:1
                obj.plotArm(obj.nodes(obj.pose_path(ii)).q);
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
            hed(1) = plot(link_cart(1,:),link_cart(2,:),'MarkerFaceColor',[0 0 1],...
                'MarkerEdgeColor',[0 0.447058826684952 0.74117648601532],...
                'Marker','o',...
                'LineWidth',5,...
                'Color',[1 0 0]);
            
            tf = tf_base_pose*obj.TF(obj.links_length(1),0,q(1))*...
                obj.TF(0,0,q(2));
            link_cart = tf * [0 , obj.links_length(2)
                0 , 0
                1 , 1];
            hed(2) = plot(link_cart(1,:),link_cart(2,:),'MarkerFaceColor',[0 0 1],...
                'MarkerEdgeColor',[0 0.447058826684952 0.74117648601532],...
                'Marker','o',...
                'LineWidth',5,...
                'Color',[1 0 0]);
            
            tf = tf_base_pose*obj.TF(obj.links_length(1),0,q(1))*obj.TF(obj.links_length(2),0,q(2))*...
                obj.TF(0,0,q(3));
            link_cart = tf * [0 , obj.links_length(3)
                0 , 0
                1 , 1];
            hed(3) = plot(link_cart(1,:),link_cart(2,:),'MarkerFaceColor',[0 0 1],...
                'MarkerEdgeColor',[0 0.447058826684952 0.74117648601532],...
                'Marker','o',...
                'LineWidth',5,...
                'Color',[1 0 0]);
            obj.hed = hed;
        end
        
    end
end
