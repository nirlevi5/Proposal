classdef DualArm2D < handle
    properties
        arm1
        arm2
        joint_path
        nodes
        opened
        closed
    end
    
    methods
        function obj = DualArm2D(arm1,arm2)
            % initialization
            obj.arm1 = arm1;
            obj.arm2 = arm2;
        end
        
        function path = FindPathAstarJoints(obj,goal,proximity,angle_res)

            %init closeset
            closedset           = [];
            L_closed            = 0;
            
            % generate initial node
            q                   = [obj.arm1.joints_value obj.arm2.joints_value];
            pose                = [obj.arm1.FK(obj.arm1.joints_value) obj.arm2.FK(obj.arm2.joints_value) ];
            g                   = 0;
            h                   = obj.calcH(goal,pose);
            f                   = g+h;
            nodelist            = struct('q',q,'pose',pose,'g',g,'h',h,'f',f,'parent',0,'id',1);
            L_nodelist          = 1;
            
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
                
                obj.plotDualArm(nodelist(current).q);
                pause(0.1)
                
                % check if we've got to goal
                norm(nodelist(current).pose(1:2) - goal(1:2)) + norm(nodelist(current).pose(4:5) - goal(4:5))
                if norm(nodelist(current).pose(1:2) - goal(1:2)) + norm(nodelist(current).pose(4:5) - goal(4:5)) <= proximity
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
                
                
                for a1q1 = -1:1
                    for a1q2 = -1:1
                        for a1q3 = -1:1
                            for a2q1 = -1:1
                                for a2q2 = -1:1
                                    for a2q3 = -1:1
                                        
                                        if ((a1q1==0) && (a1q2==0) && (a1q3==0) && (a2q1==0) && (a2q2==0) && (a2q3==0) )
                                            continue
                                        end
                                        
                                        neighbor.q      = nodelist(current).q + angle_res*[a1q1 a1q2 a1q3 a2q1 a2q2 a2q3 ];
                                        neighbor.pose   = [obj.arm1.FK(neighbor.q(1:3)) obj.arm2.FK(neighbor.q(4:6))];
                                        
                                        

                                        % check kinematic feasibility
                                        if any(neighbor.q(1:3)<obj.arm1.joints_lim(1) | neighbor.q(1:3)>obj.arm1.joints_lim(2) | ...
                                               neighbor.q(4:6)<obj.arm2.joints_lim(1) | neighbor.q(4:6)>obj.arm2.joints_lim(2) )
                                            continue;
                                        end
                                        
%                                         obj.plotDualArm(neighbor.q);
%                                         pause()
                                        
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
                                        
                                        tentative_g_score = nodelist(current).g +...
                                            norm(nodelist(current).pose(1:2) - neighbor.pose(1:2)) + ...
                                            norm(nodelist(current).pose(4:5) - neighbor.pose(4:5));
                                        
                                        % check if neighbor is in openset
                                        founded = 0;
                                        for nn = 1:L_open
                                            if all(nodelist(openset(nn)).q==neighbor.q)
                                                founded = nn;
                                                break;
                                            end
                                        end
                                        if founded==0
                                            % check for collisions in
                                            % final configuration
                                            if obj.CollisionCheck(neighbor.q)
                                                continue;
                                            end
                                            
                                            % check for collisions
                                            % during motion
                                            if obj.CollisionCheckonMotion(neighbor.q,nodelist(current).q)
                                                continue;
                                            end
                                            
                                            % add this node to nodellist
                                            neighbor.g = Inf;
                                            neighbor.h = obj.calcH(goal,neighbor.pose);
                                            neighbor.f = Inf;
                                            neighbor.parent = [];
                                            neighbor.id = L_nodelist + 1;
                                            
                                            nodelist = [nodelist ; neighbor];
                                            L_nodelist = L_nodelist + 1;
                                            
                                            % add this node to openset
                                            openset = [openset ; neighbor.id];
                                            L_open = L_open + 1;
                                            
                                            nodelist(neighbor.id).parent    = current;
                                            nodelist(neighbor.id).g         = tentative_g_score;
                                            nodelist(neighbor.id).f         = tentative_g_score + nodelist(neighbor.id).h;
                                            
                                            neighbor = [];
                                            
                                        elseif tentative_g_score <= nodelist(openset(nn)).g
                                            % this is a better path
                                            % check for collisions during motion
                                            if obj.CollisionCheckonMotion(neighbor.q,nodelist(current).q)
                                                continue;
                                            end
                                            
                                            nodelist(openset(nn)).parent    = current;
                                            nodelist(openset(nn)).g         = tentative_g_score;
                                            nodelist(openset(nn)).f         = tentative_g_score + nodelist(openset(nn)).h;
%                                             continue;
                                        end
                                        
                                    end
                                end
                            end
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
        
        function H = calcH(obj,goal,pose)
            H = norm(goal(1:2)-pose(1:2)) + ...
                norm(goal(4:5)-pose(4:5));
        end
        
        function flag = CollisionCheck(obj,q)
            
            if nargin==1
                q = [ obj.arm1.joints_value , obj.arm2.joints_value ];
            end
            
            [L11 , L12 , L13] = obj.arm1.calcLinksVertex(q(1:3));
            [L21 , L22 , L23] = obj.arm2.calcLinksVertex(q(4:6));
            
            if sat(L13',L21')
                flag = 1;
                return;
            elseif sat(L13',L22') 
                flag = 1;
                return;
            elseif sat(L13',L23') 
                flag = 1;
                return;
            elseif sat(L12',L21')
                flag = 1;
                return; 
            elseif sat(L12',L22')
                flag = 1;
                return; 
            elseif sat(L12',L23') 
                flag = 1;
                return;
            elseif sat(L11',L21') 
                flag = 1;
                return;
            elseif sat(L11',L22') 
                flag = 1;
                return;
            elseif sat(L11',L23')
                flag = 1;
                return;
            else
                flag = 0;
            end
        end
        
        function flag = CollisionCheckonMotion(obj,current_q,prior_q)
            
            current_av1 = obj.arm1.calcArmVertex(current_q(1:3));
            prior_av1   = obj.arm1.calcArmVertex(prior_q(1:3));
            
            current_av2 = obj.arm2.calcArmVertex(current_q(4:6));
            prior_av2   = obj.arm2.calcArmVertex(prior_q(4:6));
            
            rect13 = [prior_av1(3:4,:) ; flipud(current_av1(3:4,:))];
            % arm1 link 3 with arm2 link 3
            rect23 = [prior_av2(3:4,:) ; flipud(current_av2(3:4,:))];
            if sat(rect13,rect23)
                flag = 1;
                return;
            end
            % arm1 link 3 with arm2 link 2
            rect22 = [prior_av2(2:3,:) ; flipud(current_av2(2:3,:))];
            if sat(rect13,rect22)
                flag = 1;
                return;
            end
            % arm1 link 3 with arm2 link 1
            rect21 = [prior_av2(1:2,:) ; flipud(current_av2(1:2,:))];
            if sat(rect13,rect21)
                flag = 1;
                return;
            end
            
            rect12 = [prior_av1(2:3,:) ; flipud(current_av1(2:3,:))];
            % arm1 link 2 with arm2 link 1
            if sat(rect12,rect21)
                flag = 1;
                return;
            end
            % arm1 link 2 with arm2 link 2
            if sat(rect12,rect22)
                flag = 1;
                return;
            end
            % arm1 link 2 with arm2 link 3
            if sat(rect12,rect23)
                flag = 1;
                return;
            end
            
            rect11 = [prior_av1(1:2,:) ; flipud(current_av1(1:2,:))];
            % arm1 link 1 with arm2 link 1
            if sat(rect11,rect21)
                flag = 1;
                return;
            end
            % arm1 link 1 with arm2 link 2
            if sat(rect11,rect22)
                flag = 1;
                return;
            end
            % arm1 link 1 with arm2 link 3
            if sat(rect11,rect23)
                flag = 1;
                return;
            end
            
            flag = 0;
            
        end
        
        function plotDualArm(obj,q)
           if nargin>1
                obj.arm1.plotArm(q(1:3));
                obj.arm2.plotArm(q(4:6));
           else
                obj.arm1.plotArm();
                obj.arm2.plotArm();
           end
        end
        
        function plotDualArmPath(obj)
            for ii = length(obj.joint_path):-1:1
                obj.plotDualArm(obj.nodes(obj.joint_path(ii)).q);
                pause(0.3);
            end
        end
        
    end
end
