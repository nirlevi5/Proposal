classdef MultiArm < handle
    properties
        arms
        num_of_arms
        obstacles
        current_joints_state
        plan
        arms_sequence
        execution_series
        ConnGraph
        hed_tp
        joints_res
        max_proximity % maximum proximity between arms
        task
    end
    
    methods
        function obj = MultiArm(arms)
            %% initialization
            obj.arms            = arms;
            obj.num_of_arms     = length(arms);
            obj.max_proximity   = 0.05; 
            obj.joints_res      = 0.05;
%             obj.obstacle_map   = [];
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
        
        function  GenerateMotionPlan(obj,task)
            % update collision map
            obj.obstacles.pose      = task.object_position(1:2);
            obj.obstacles.vertexes  = repmat(task.object_position(1:2)',[4,1]) + [-0.05 0.05 0.05 -0.05 ; 0.05 0.05 -0.05 -0.05]'; 
            
            queue(1).task = task;
            
            while (length(queue)>=1 )
                % generate theoretic path
                plan = obj.FindAttracivePath(queue(1).task);
                % check if no arms obstruct the plan
                plan = obj.ValidatePlan(plan);
                if plan.solution
                    % execute plan
                    obj.plotPlan(plan);
                    % update new configuration
                    for ii = 1:obj.num_of_arms
                        obj.arms(ii).joints_value = plan.last_arms_state(ii).value;
                    end
                    % remove task from queue
                    queue(1) = [];
                else
                    % generate sub task
                    plan.path = flipud(plan.path);
                    plan.pre_path = [];
                    plan = obj.ValidatePlan(plan);
                
                    if plan.armID == 1
                        subtask.task.armID           = 2;
                    else
                        subtask.task.armID           = 1;
                    end
                    subtask.task.target_joints_value    = plan.last_arms_state(subtask.task.armID).value;
                    queue = [subtask ; queue];
                end
            
            end
            obj.plan = plan;
        end
       
        function plan = FindAttracivePath(obj,task)
            % task.armID                  = 1;
            % task.target_joints_value    = [1 -1];
            % returns a joints motion plan for the arm in the task
            % after this function finished a collision check is applied in
            % ValidateExecutionPath function
            
            plan.solution           = 0; 
            plan.path               = obj.arms(task.armID).joints_value;
            plan.armID              = task.armID;
            plan.sub_plan           = [];
            plan.repulsiveArms      = [];
            plan.initial_arms_state = [];
            
            % initiatial arms state
            for ii = 1:obj.num_of_arms
                plan.initial_arms_state(ii).value = obj.arms(ii).joints_value;
            end
            
            while ~plan.solution
                
                branching = repmat(plan.path(end,:) , [obj.arms(plan.armID).branching_factor,1]) ...
                    + obj.joints_res * obj.arms(plan.armID).branching_matrix;
                
                % calc heuristics as the sum of delta joints
                for jj = 1:size(branching,1)
                    h(jj) = sum(abs(branching(jj,:) - task.target_joints_value));
                end

                % sort the heuristics by ascending order
                [asc_h,asc_ind] = sort(h);
                
                % generate arm motion
                for ii = 1:length(asc_ind)
                    % check for joints limitation
                    if any(branching(asc_ind(ii),:)<obj.arms(plan.armID).joints_lim(1))
                        continue;
                    elseif any(branching(asc_ind(ii),:)>obj.arms(plan.armID).joints_lim(2))
                        continue;
                    end
                    
                    % execution
                    plan.path(end+1,:) = branching(asc_ind(ii),:);
                    if norm(plan.path(end,:) - task.target_joints_value)<10e-10
                        % solution found
                        plan.solution = 1;
                    end
                    break;
                end
            end
        end
        function plan = FindRepulsivePath(obj,task)
            % task.armID                        = 2;
            % task.initial_state                = [1 -1]
            % task.repulsiveArms().id              = 1;
            % task.repulsiveArms().state           = [1 -1];
            % returns a joints motion plan for the arm in the task
            % after this function finished a collision check is applied in
            % ValidateExecutionPath function
            
            plan.armID              = task.armID;
            plan.path               = task.initial_arms_state(task.armID).value;
            plan.solution           = 0;
            plan.sub_plan           = [];
            plan.repulsiveArms      = task.repulsiveArms(:);
            plan.initial_arms_state = task.initial_arms_state
            
            while ~plan.solution
                
                branching = repmat(plan.path(end,:) , [obj.arms(plan.armID).branching_factor,1]) ...
                    + obj.joints_res * obj.arms(plan.armID).branching_matrix;
                
                % calc heuristics as min distance between repulsive arms
                % and the moveing arm (plan.armID)
                for ii = 1:size(branching,1)
                    % check with each of the repulsive arms
                    h(ii) = 100000;
                    for jj = 1:length(task.repulsiveArms)
                        if obj.ArmArmCollisionCheck(plan.armID,task.repulsiveArms(jj).id,branching(ii,:),task.repulsiveArms(jj).state)
                            h(ii) = 0;
                            break;
                        else
                            % calculate min distance
                            h_temp = obj.calcArmsMinDistance(plan.armID,task.repulsiveArms(jj).id,branching(ii,:),task.repulsiveArms(jj).state); ;
                            if h_temp<h(ii)
                                h(ii) = h_temp;
                            end
                        end
                    end
                end

                % sort the heuristics by decending order ()
                [des_h,des_ind] = sort(h,'descend');
                
                % generate arm motion
                for ii = 1:length(des_ind)
                    % check for joints limitation
                    if any(branching(des_ind(ii),:)<obj.arms(plan.armID).joints_lim(1))
                        continue;
                    elseif any(branching(des_ind(ii),:)>obj.arms(plan.armID).joints_lim(2))
                        continue;
                    end
                    
                    % check for collision with the object
                    flag = obj.ArmObsCollisionCheck(plan.armID,1,branching(des_ind(ii),:));
                    if flag
                        continue
                    end
                        
                    % no collision and no joints limits violation, execute
                    if des_h(ii) == 0 % state in collision
                        return
                    elseif all(plan.path(end,:) == branching(des_ind(ii),:)) % the same state as the previuse
                        % solution found but without respecting
                        % max_proximity rule
                        plan.solution = 1;
                        break;
                    else
                        plan.path(end+1,:) = branching(des_ind(ii),:);
                    end
                    
                    if des_h(ii)>obj.max_proximity
                        % solution found
                        plan.solution = 1;
                    end
                    break;
                end
            end
        end
        function plan = ValidatePlan(obj,plan)
            % validate the motion described in plan
            % apply a repulsive function on each step that cause a
            % collision (or in proximity to a collision)

            arms_current_state = plan.initial_arms_state;
            
           % go through each step of execution path and find a sub-plans for the other arms 
            for ii = 1:size(plan.path,1)
                plan.sub_plan(ii).plan = [];
                % check for proximity/collision of the moving arm with each of the other arms
                for jj = 1:obj.num_of_arms
                    % ignore same arm check 
                    if plan.armID == jj
                        mindistance(jj) = 200;
                        continue
                    end
                    
                    % ignore repulsive arm check 
                    for kk = 1: length(plan.repulsiveArms)
                        if plan.repulsiveArms(kk).id == jj
                            mindistance(jj) = 100;
                            continue;
                        end
                    end
                    
                    % calculate min distance
                    if obj.ArmArmCollisionCheck(plan.armID,jj,plan.path(ii,:),arms_current_state(jj).value)
                        mindistance(jj) = 0;
                    else
                        mindistance(jj) = obj.calcArmsMinDistance(plan.armID,jj,plan.path(ii,:),arms_current_state(jj).value) ;
                    end
                     
                end
                
                % find the arm with the minimal distance
                [s_min_d,ind_min_d] = sort(mindistance);
                
                % check if step ii of execution path has no collision or far enough from the moving arm
                if s_min_d(1)>obj.max_proximity
                    arms_current_state(plan.armID).value    = plan.path(ii,:);
                    plan.sub_plan(ii).plan                  = [];
                    continue; % go to step ii+1 of execution path
                end
                
                % ToDo - handle a case with 3 arms in collision
                
                % move the arm with the minimal distance
                newtask.armID               = ind_min_d(1);
                newtask.initial_arms_state  = arms_current_state;
                  repulsiveArm.id           = plan.armID;
                  repulsiveArm.state        = plan.path(ii,:);
                newtask.repulsiveArms       = [plan.repulsiveArms ; repulsiveArm];
                subplan                     = obj.FindRepulsivePath(newtask);
                
                subplan = obj.ValidatePlan(subplan);
                if subplan.solution
                    arms_current_state                          = subplan.last_arms_state;
                    arms_current_state(plan.armID).value        = plan.path(ii,:);
                    plan.sub_plan(ii).plan                      = subplan;
                else
                    plan.solution = 0;
                    return
                end
                
            end
            plan.solution = 1;
            plan.last_arms_state = arms_current_state;
        end
        
         
        function min_d = calcArmsMinDistance(obj,ID1,ID2,q1,q2)
            % calcs minimum distance between two arms
            
            arm1links = obj.arms(ID1).calcJointsPosition(q1);
            arm2links = obj.arms(ID2).calcJointsPosition(q2);
            
            min_d = inf;
            for ii = 1:obj.arms(ID1).number_of_links
                for jj = 1:obj.arms(ID2).number_of_links
                    d = obj.calcLinksMinDistance(arm1links(ii:ii+1,:),arm2links(jj:jj+1,:));
                    if d<min_d
                        min_d = d;
                    end
                end
            end
                
        end
        function min_d = calcLinksMinDistance(obj,L1,L2)
            % minimum distace between two links
            % link is given with 2 global points 
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
            % minimum distance between point and link
            % link is given by 2 global points
            
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
       
        
        function flag = ArmArmCollisionCheck(obj,masterArmID,slaveID,masterArm_q,slaveArm_q)
            % returns 1 if collision exist between two arms
            if nargin>4
                L1 = obj.arms(masterArmID).calcLinksVertexs(masterArm_q);
                L2 = obj.arms(slaveID).calcLinksVertexs(slaveArm_q);
            elseif nargin>3
                L1 = obj.arms(masterArmID).calcLinksVertexs(masterArm_q);
                L2 = obj.arms(slaveID).calcLinksVertexs();
            else
                L1 = obj.arms(masterArmID).calcLinksVertexs();
                L2 = obj.arms(slaveID).calcLinksVertexs();
            end
            
            for ii = 1:obj.arms(masterArmID).number_of_links;
                for jj = 1:obj.arms(slaveID).number_of_links;
                    if sat(L1(:,:,ii)',L2(:,:,jj)')
                        flag = 1;
                        return;
                    else
                        flag = 0;
                    end
                end
            end
            
        end  
        function flag = ArmObsCollisionCheck(obj,ArmID,ObsID,Arm_q)
            % returns 1 if collision exist between arm and obstacle
            if nargin>3
                L1 = obj.arms(ArmID).calcLinksVertexs(Arm_q);
                L2 = obj.obstacles(ObsID).vertexes;
            else
                L1 = obj.arms(ArmID).calcLinksVertexs();
                L2 = obj.obstacles(ObsID).vertexes;
            end
            
            for ii = 1:obj.arms(ArmID).number_of_links;
                if sat(L1(:,:,ii)',L2)
                    flag = 1;
                    return;
                else
                    flag = 0;
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
        function q = plotPlan(obj,plan)
            
            if nargin<2  , plan=obj.plan;  end
            
            for ii=1:obj.num_of_arms
                q(ii,:) = plan.initial_arms_state(ii).value;
            end
%             
            obj.plotAllArms(q);
            plot(obj.obstacles.pose(1),obj.obstacles.pose(2),'om','MarkerSize',15,'Linewidth',3);
            
            % plot each step of the plan
            for ii = 1:size(plan.path,1)
                if ~isempty(plan.sub_plan(ii).plan)
                    q = obj.plotPlan(plan.sub_plan(ii).plan);
                end
                % this is a valid move, no pre path exist
                q(plan.armID,:)=plan.path(ii,:);
                obj.plotAllArms(q);
                pause(0.05);
                continue;
            end
        end
        function plotPlanold(obj,plan)
            
            if nargin<2
                plan=obj.plan;
            end
            
            for ii=1:obj.num_of_arms
                q(ii,:) = obj.arms(ii).joints_value;
            end
            
            obj.plotAllArms(q);
            plot(obj.obstacles.pose(1),obj.obstacles.pose(2),'om','MarkerSize',15,'Linewidth',3);
            % plot each step of the plan
            for ii = 1:size(plan.path,1)
                if size(plan.pre_path(ii).arm,1)==0
                    % this is a valid move, no pre path exist
                    q(plan.armID,:)=plan.path(ii,:);
                    obj.plotAllArms(q);
                    pause(0.1);
                    continue; 
                end
                % plot the pre_path of step ii
                ind = 1;
                update = 1;
                while (update)
                    update = 0;
                    for jj = 1:obj.num_of_arms
                        % ignore the leading arm
                        if jj==plan.armID
                            continue;
                        end
                        
                        if size(plan.pre_path(ii).arm(jj).path,1)<ind
                            continue;
                        else
                            q(jj,:) = plan.pre_path(ii).arm(jj).path(ind,:);
                            update = 1;
                        end
                    end
                    if update
                        obj.plotAllArms(q);
                        pause(0.1);
                    end
                    ind = ind + 1;
                end
                q(plan.armID,:)=plan.path(ii,:);
                obj.plotAllArms(q);
                pause(0.1);
            end
        end
        function plot_transfer_points(obj,seq_ind)
            tp = obj.plan.sequences(seq_ind).tp;
            obj.hed_tp = plot(tp(1,:),tp(2,:),'m*-','LineWidth',2);
        end
    end % method
    
end % class

%                     flag = obj.CollisionCheck(execution_path.armID,jj,execution_path.values(ii,:),arms_current_state(ii,:));
%                     if flag % collision found
%                         
%                     end
