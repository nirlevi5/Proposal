classdef MultiArmSingleLink < handle
    properties
        arms
        num_of_arms
        plan
        arms_sequence
        ConnGraph
        hed_tp
    end
    
    methods
        function obj = MultiArmSingleLink(arms)
            %% initialization
            obj.arms = arms;
            obj.num_of_arms = length(arms);
            
            %% generate connectivity graph
            % initiate connctivity graph
            for ID = 1:obj.num_of_arms
                obj.ConnGraph(ID).neighbors = [];
            end
            % fill in connectivity graph
            for ID1 = 1:obj.num_of_arms
                for ID2 = ID1+1 : obj.num_of_arms
                    if sqrt((obj.arms(ID1).base_pose.x-obj.arms(ID2).base_pose.x)^2 ...
                            +(obj.arms(ID1).base_pose.y-obj.arms(ID2).base_pose.y)^2 )...
                            < (sum(obj.arms(ID1).links_length) + sum(obj.arms(ID2).links_length))
                        obj.ConnGraph(ID1).neighbors(end+1) = ID2;
                        obj.ConnGraph(ID2).neighbors(end+1) = ID1;
                    end
                end
            end
            
        end
        
        function arms_sequence = calc_arms_sequence(obj,task)
            Initial_pose    = task.init;
            target_pose     = task.target;
            % initiate an optional final and initial arms sets
            initial_arm_set = [];
            final_arm_set   = [];
            for ID = 1:obj.num_of_arms
                % check if arm can be initial arm
                if norm([obj.arms(ID).base_pose.x ; obj.arms(ID).base_pose.y] - ...
                        Initial_pose) < sum(obj.arms(ID).links_length)
                    initial_arm_set = [initial_arm_set ; ID];
                end
                
                % check if arm can be final arm
                if norm([obj.arms(ID).base_pose.x ; obj.arms(ID).base_pose.y] - ...
                        target_pose) < sum(obj.arms(ID).links_length)
                    final_arm_set = [final_arm_set ; ID];
                end
                
            end
             
            %% find sequence using graph search
            % generate all sequences
            for ii = 1:length(initial_arm_set)
                openset(ii) = struct('ID',initial_arm_set(ii),'parent',0);
            end
            
            nodeset = [];
            sequence_ID = 1;
            while ~isempty(openset)
                if any(openset(1).ID == final_arm_set)
                    % sequence found
                    all_sequences(sequence_ID).arms = fliplr(GenerateSequence(nodeset,openset(1))) ;
                    openset = openset(2:end);
                    sequence_ID = sequence_ID + 1;
                    continue;
                end
                nodeset = [nodeset ; openset(1)];
                
                for ii = 1:length(obj.ConnGraph(openset(1).ID).neighbors)
                    % check if arm is already in this sequence
                    node = struct('ID',obj.ConnGraph(openset(1).ID).neighbors(ii),'parent',length(nodeset));
                    arm2check = node;
                    flag = 0;
                    while (arm2check.parent~=0)
                        if nodeset(arm2check.parent).ID ~= node.ID
                            arm2check = nodeset(arm2check.parent);
                        else
                            flag = 1;
                            break
                        end
                    end
                    if ~flag
                        openset(end+1) = node;
                    end
                end
                openset = openset(2:end);
            end
            
            obj.plan.sequences  = all_sequences;
            obj.plan.task       = task;
            
            function res = GenerateSequence(nodeset,init_node)
                current = init_node;
                res = [];%init_node.ID;
                while 1
                    res(end+1) = current.ID;
                    if current.parent ~= 0
                        current = nodeset(current.parent);
                    else
                        break;
                    end
                end
            end

        end
        
        function calc_transfer_points(obj)
            for ii = 1:length(obj.plan.sequences)
                sequence = obj.plan.sequences(ii).arms;
                tp = [];
                for jj = 1:length(sequence)-1
                    p1 = [obj.arms(sequence(jj)).base_pose.x  ; obj.arms(sequence(jj)).base_pose.y];
                    p2 = [obj.arms(sequence(jj+1)).base_pose.x ; obj.arms(sequence(jj+1)).base_pose.y];
                    d  = norm(p1-p2) ;
                    r1 = sum(obj.arms(sequence(jj)).links_length);
                    r2 = sum(obj.arms(sequence(jj+1)).links_length);
                    
                    tp = [ tp ,(p1*((d-r1+r2)/d) + p2*((d+r1-r2)/d))/2];
                end
                obj.plan.sequences(ii).tp = [obj.plan.task.init , tp , obj.plan.task.target];
            end
        end
        
        function sequence_trajectory = calc_sequence_trajectory(arms_sequence,transfer_points)
            
        end
        
        function plan = FindPathMultiArm(obj,goal,proximity,angle_res)
            plan.arms(1).ID      = goal.armID;
            plan.arms(1).path    = obj.arms(goal.armID).FindPathAstar(goal.pose,proximity,angle_res);
            
            flag = obj.VerifySingleArmMotion(plan); % 1 means no success
            if flag
                obj.plan = obj.VerifyRepulsiveMotion(plan);
            end
        end
        
        function flag = VerifySingleArmMotion(obj,plan)
            
            masterArmID     = plan.arms(1).ID;
            masterArmPath   = plan.arms(1).path;
            slaveArmID      = 2;
            for ii = 1:length(masterArmPath)
                flag = obj.CollisionCheck(masterArmID,masterArmPath(ii),slaveArmID);
                if flag
                    return
                end
            end
            
        end
        
        function plan = VerifyRepulsiveMotion(obj,plan)
            
            masterArmID     = plan.arms(1).ID;
            masterArmPath   = plan.arms(1).path;
            slaveArmID      = 2;
            
            masterArmVert = obj.arms(masterArmID).calcArmVertexs(masterArmPath(1,:));
            slaveArmVert = obj.arms(slaveArmID).calcArmVertexs();
            d = obj.calcMinDistance(masterArmVert,slaveArmVert);
            
            joints = obj.arms(slaveArmID).joints_value;
            while d<0.05
                sav1 = obj.arms(slaveArmID).calcArmVertexs(joints + obj.arms(slaveArmID).joints_res);
                d1 = obj.calcMinDistance(masterArmVert,sav1)
                sav2 = obj.arms(slaveArmID).calcArmVertexs(joints - obj.arms(slaveArmID).joints_res);
                d2 = obj.calcMinDistance(masterArmVert,sav2)
                
                if d1>d2
                    joints = joints + obj.arms(slaveArmID).joints_res;
                    d = d1;
                else
                    joints = joints - obj.arms(slaveArmID).joints_res;
                    d = d2;
                end
            end
            
            slaveArmPath = joints;
            for ii = 2:length(masterArmPath)
                masterArmVert = obj.arms(masterArmID).calcArmVertexs(masterArmPath(ii,:));
                slaveArmVert = obj.arms(slaveArmID).calcArmVertexs(joints);
                d = obj.calcMinDistance(masterArmVert,slaveArmVert)
                joints = slaveArmPath(end,:);
                while d<0.05
                    sav1    = obj.arms(slaveArmID).calcArmVertexs(joints + obj.arms(slaveArmID).joints_res);
                    d1      = obj.calcMinDistance(masterArmVert,sav1);
                    sav2    = obj.arms(slaveArmID).calcArmVertexs(joints - obj.arms(slaveArmID).joints_res);
                    d2      = obj.calcMinDistance(masterArmVert,sav2);
                    
                    if d1>d2
                        joints = joints + obj.arms(slaveArmID).joints_res;
                        d = d1;
                    else
                        joints = joints - obj.arms(slaveArmID).joints_res;
                        d = d2;
                    end
                end
                slaveArmPath = [slaveArmPath ; joints];
                
            end
            
            plan.arms(2).ID = slaveArmID;
            plan.arms(2).path = slaveArmPath;
        end
        
        
        function min_d = calcMinDistance(obj,L1,L2)
            
            p1 = L1(1,:);
            p2 = L1(2,:);
            p3 = L2(1,:);
            p4 = L2(2,:);
            
            d1 = obj.min_distance_point_to_link(p1,L2);
            d2 = obj.min_distance_point_to_link(p2,L2);
            d3 = obj.min_distance_point_to_link(p3,L1);
            d4 = obj.min_distance_point_to_link(p4,L1);
            
            min_d = min([d1,d2,d3,d4]);
            
        end
        
        function d = min_distance_point_to_link(obj,p,L)
            % check p1 with p3 and p4
            ang1 = acos(dot(p-L(1,:),L(2,:)-L(1,:))/(norm(p-L(1,:))*norm(L(2,:)-L(1,:))));
            ang2 = acos(dot(p-L(2,:),L(1,:)-L(2,:))/(norm(p-L(2,:))*norm(L(1,:)-L(2,:))));
            
            if (ang1>pi/2)
                d = norm(p-L(1,:));
            elseif ang2>pi/2
                d = norm(p-L(2,:));
            else
                d = abs( p(1)*(L(2,2) - L(1,2)) + ...
                    L(1,1)*(p(2) - L(2,2)) + ...
                    L(2,1)*(L(1,2) - p(2)) )...
                    /norm(L(1,:)-L(2,:));
            end
        end
        
        function flag = CollisionCheck(obj,masterArmID,masterArmQ,slaveID)
            
            L1 = obj.arms(masterArmID).calcLinksVertexs(masterArmQ);
            L2 = obj.arms(slaveID).calcLinksVertexs();
            
            if sat(L1',L2')
                flag = 1;
                return;
            else
                flag = 0;
            end
        end
        
        
        %% PLOTS
        
        function plotAllArms(obj,q)
            if nargin>1
                for ii = 1:obj.num_of_arms
                    obj.arms(ii).plotArm(q(ii,:));
                end
            else
                for ii = 1:obj.num_of_arms
                    obj.arms(ii).plotArm();
                end
            end
        end
        
        function plotAllWorkspaces(obj)
            for ii = 1:obj.num_of_arms
                obj.arms(ii).plotWorkSpace();
            end
        end
        
        function plotPlan(obj)
            
            for ii = 1:length(obj.plan.arms(1).path) % path index
                for jj = 1:length(obj.plan.arms) % arm index
                    obj.arms(obj.plan.arms(jj).ID).plotArm(obj.plan.arms(jj).path(ii));
                    pause(0.3);
                end
            end
            
        end
        
        function plot_transfer_points(obj,seq_ind)
           tp = obj.plan.sequences(seq_ind).tp;
           obj.hed_tp = plot(tp(1,:),tp(2,:),'m*-','LineWidth',2);
        end
    end % method
    
end % class
