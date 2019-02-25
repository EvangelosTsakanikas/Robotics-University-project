clear variables; % Remove all variables from the workspace.
close all; % Close all open figure windows.

% position of legs (base)
leg1_base = [-1; -1; 0];
leg2_base = [ 1; -1; 0];
leg3_base = [ 0;  1; 0];
legsPositions_base = [leg1_base, leg2_base, leg3_base];
% position of legs (platform)
leg1_platform = [-0.5; -0.5; 0];
leg2_platform = [ 0.5; -0.5; 0];
leg3_platform = [ 0.0;  0.5; 0];
legsPositions_platform = [leg1_platform, leg2_platform, leg3_platform];

option = input(['Press 1 to solve the Inverse Kinematics problem or \n'...
                'Press 2 to solve the Forward Kinematics problem or \n'...
                'Press 3 to solve the Inverse Differential Kinematics problem or \n'...
                'Press 4 to solve the Differential Kinematics problem or \n'...
                'Press 5 to construct platform\`s orbit \n: ']);

while (option ~= 0)
    
    %%%%% Inverse Kinematics
    if (option == 1)        
        % read from keyboard for inverse Kinematics
        P_x = input('Give the x coordinate of the platform\`s destination (P_x): ');
        P_y = input('Give the y coordinate of the platform\`s destination (P_y): ');
        P_z = input('Give the z coordinate of the platform\`s destination (P_z): ');
        fprintf('\n');
        % end of input reading

        P = [P_x, P_y, P_z];

        % calculate inverse Kinematics problem and plot
        inverseKinematics_solution = InverseKinematics(legsPositions_base,...
                                                       legsPositions_platform, P);

        if (CheckLegLength(inverseKinematics_solution) == 1)
            leg1_platform_afterMoving = transpose(P) + leg1_platform;
            leg2_platform_afterMoving = transpose(P) + leg2_platform;
            leg3_platform_afterMoving = transpose(P) + leg3_platform;
            legsPositions_afterMoving = [leg1_platform_afterMoving, leg2_platform_afterMoving, leg3_platform_afterMoving];

            legs_length = inverseKinematics_solution();

            DoThePlot(legsPositions_base, legsPositions_afterMoving, legs_length, P, []);
        end 
    end

    %%%%% Forward Kinematics
    if (option == 2)         
        % read from keyboard for forward Kinematics
        d1 = input('Give the length of the 1st leg: ');
        d2 = input('Give the length of the 2nd leg: ');
        d3 = input('Give the length of the 3rd leg: ');
        fprintf('\n');
        % end of input reading
        legs_length = [d1; d2; d3];
        
        if (CheckLegLength(legs_length) == 1)            
            % calculate forward Kinematics problem and plot
            forwardKinematics_solution = ForwardKinematics(legsPositions_base,...
                                                           legsPositions_platform, legs_length);

            if (forwardKinematics_solution ~= -1)
                leg1_platform_afterMoving = leg1_platform + forwardKinematics_solution;
                leg2_platform_afterMoving = leg2_platform + forwardKinematics_solution;
                leg3_platform_afterMoving = leg3_platform + forwardKinematics_solution;
                legsPositions_afterMoving = [leg1_platform_afterMoving, leg2_platform_afterMoving, leg3_platform_afterMoving];

                DoThePlot(legsPositions_base, legsPositions_afterMoving, legs_length, forwardKinematics_solution, []);
            else
                fprintf('\n\n');
            end
        end
    end
    
    %%%%% Inverse Differential Kinematics
    if (option == 3)
        % read from keyboard for inverse differential Kinematics
        P_x = input('Give the x coordinate of the platform\`s destination (P_x): ');
        P_y = input('Give the y coordinate of the platform\`s destination (P_y): ');
        P_z = input('Give the z coordinate of the platform\`s destination (P_z): ');
        fprintf('\n');
        V_x = input('Give the velocity on (x-axis) (V_x): '); 
        V_y = input('Give the velocity on (y-axis) (V_y): ');
        V_z = input('Give the velocity on (z-axis) (V_z): ');        
        fprintf('\n');
        % end of input reading
        
        P = [P_x; P_y; P_z];
        Vp = [V_x; V_y; V_z];
        
        q_dot = InverseDifferentialKinematics(legsPositions_base,...
                                              legsPositions_platform,...
                                              P, Vp); 
        
        if (q_dot ~= -1)
            fprintf('q_dot = \n\t\t%f\n\t\t%f\n\t\t%f \n\n' ,q_dot(1), q_dot(2), q_dot(3));
        end
    end
    
    %%%%% Differential Kinematics    
    if (option == 4)
        % read from keyboard for forward Kinematics
        d1 = input('Give the length of the 1st leg: ');
        d2 = input('Give the length of the 2nd leg: ');
        d3 = input('Give the length of the 3rd leg: ');
        fprintf('\n');       
        d_dot_1 = input('Give d_dot of leg1: '); 
        d_dot_2 = input('Give d_dot of leg2: ');
        d_dot_3 = input('Give d_dot of leg3: ');
        fprintf('\n');
        % end of input reading
        
        legs_length = [d1; d2; d3];
        q_dot = [d_dot_1; d_dot_2; d_dot_3];
        
        if (CheckLegLength(legs_length) == 1)            
            
            Ve = DifferentialKinematics(legsPositions_base,...
                                        legsPositions_platform, legs_length, q_dot);
        end
        
        if (Ve ~= -1)
            fprintf('Vp = \n\t\t%f\n\t\t%f\n\t\t%f \n\n' ,Ve(1), Ve(2), Ve(3));
        end
    end
    
    %%%%% Orbit
    if (option == 5)
        
        questionToTheUser = input(['Press 1 for trajectory planning or \n'...
                                   'Press !=1 to load default trajectory \n: ']);                                     
        orbit = [];
        Velocity = [];
        if (questionToTheUser == 1) % trajectory planning
            
            numberOfOrbitPoints = input('Give the nubmer of Orbit points: ');
               
            for i=1:numberOfOrbitPoints
                fprintf('point %d \n', i);
                P_x = input('Give the x coordinate of the platform\`s orbit point (P_x): ');
                P_y = input('Give the y coordinate of the platform\`s orbit point (P_y): ');
                P_z = input('Give the z coordinate of the platform\`s orbit point (P_z): ');
                fprintf('\n');
                P = [P_x P_y P_z];
                orbit = cat(1, orbit, P);
                
                if (numberOfOrbitPoints > 2)
                    fprintf('point %d \n', i);
                    V_x = input('Give the velocity on (x-axis) of this platform\`s orbit point (V_x): '); 
                    V_y = input('Give the velocity on (y-axis) of this platform\`s orbit point (V_y): ');
                    V_z = input('Give the velocity on (z-axis) of this platform\`s orbit point (V_z): ');
                    fprintf('\n');
                    V = [V_x V_y V_z];
                    Velocity = cat(1, Velocity, V);
                end               
            end  
            orbit = transpose(orbit);
            Velocity = transpose(Velocity);
            tf = input('Give tf: '); 
            
        else % default 
            orbit = [0 1 -2 0
                     0 1 -1 0
                     1 2  2 1];
 
            Velocity = [0, 0.5, 0.8, 0; 
                        0, 0.5, 0.8, 0; 
                        0, 0.5, 0.1, 0];
            tf = 3;
        end
        
        pointsThatCanBeReached = 0;
        for j=1:size(orbit, 2)
            
            inverseKinematics_solution = InverseKinematics(legsPositions_base,...
                                                           legsPositions_platform,...
                                                           orbit(:, j));
            if (CheckLegLength(inverseKinematics_solution) == 1)
               pointsThatCanBeReached = pointsThatCanBeReached + 1;
            end            
        end
        
        if (pointsThatCanBeReached == size(orbit, 2))
            PlatformOrbitCalculation(orbit, tf,...
                                     Velocity,...
                                     legsPositions_base,...
                                     legsPositions_platform);
        else
            fprintf('One of the trajectory points cannot be reached \n\n');     
        end
    end
    
    option = input(['Press 1 to solve the Inverse Kinematics problem or \n'...
                        'Press 2 to solve the Forward Kinematics problem or \n'...
                        'Press 3 to solve the Inverse Differential Kinematics problem or \n'...
                        'Press 4 to solve the Differential Kinematics problem or \n'...
                        'Press 5 to construct platform\`s orbit \n: ']);
                    
    if (option > 5 || option < 0)
        break;
    end
end