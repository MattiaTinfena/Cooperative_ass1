classdef ActionManager < handle
    properties
        actions = {}      % cell array of actions (each action = stack of tasks)
        action_names = {}
        current_action = 1 % index of currently active action
        previous_action = 1
        action_changes = 0
        action_switch_time = {}
        all_task_list = {}
    end

    methods
        function addAction(obj, taskStack, name)
            % taskStack: cell array of tasks that define an action
            obj.actions{end+1} = taskStack;
            obj.action_names{end+1} = name;
        end

        function setTaskList(obj, task_list)
            for i = 1:length(task_list)
                obj.all_task_list{end+1} = task_list{i};
            end
            disp(obj.all_task_list)
        end

        function [v_nu, qdot] = computeICAT(obj, robot)
            % Get current action
            actTasks  = obj.actions{obj.current_action};
            prevTasks = obj.actions{obj.previous_action};

            tasks = actTasks;

            if (obj.action_changes)
   
                % %update ap
                % ap = {}

                % %when gaussian transitory is ended
                obj.action_changes = 0;
                % disp(tasks);
            end


            % 1. Update references, Jacobians, activations
            for i = 1:length(tasks)
                tasks{i}.updateReference(robot);
                tasks{i}.updateJacobian(robot);
                tasks{i}.updateActivation(robot);
            end

            % 2. Perform ICAT (task-priority inverse kinematics)
            ydotbar = zeros(13,1);
            Qp = eye(13);
            for i = 1:length(tasks)
                [Qp, ydotbar] = iCAT_task(tasks{i}.A, tasks{i}.J, ...
                                           Qp, ydotbar, tasks{i}.xdotbar, ...
                                           1e-4, 0.01, 10);
            end

            % 3. Last task: residual damping
            [~, ydotbar] = iCAT_task(eye(13), eye(13), Qp, ydotbar, zeros(13,1), 1e-4, 0.01, 10);

            % 4. Split velocities for vehicle and arm
            qdot = ydotbar(1:7);
            v_nu = ydotbar(8:13); % projected on the vehicle frame
        end

    function setCurrentAction(obj, actionName)

        found = false;
        obj.previous_action = obj.current_action;
        obj.action_changes = 1;
        for i = 1:length(obj.action_names)
            if strcmp(obj.action_names{i}, actionName)
                obj.current_action = i;
                found = true;
                break; % esci dal ciclo
            end
        end

        if ~found
            error('Action not found');
        end
    end

    end
end