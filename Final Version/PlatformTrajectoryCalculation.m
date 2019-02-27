function PlatformTrajectoryCalculation(trajectory, tf,...
                                  Velocity,...
                                  legsPositions_base,...
                                  legsPositions_platform)
           
for j=1:(size(trajectory, 2)-1)

    if (size(trajectory, 2) > 2)
        % Polynom for x (X-axis)
        ao_x = trajectory(1,j);
        a1_x = Velocity(1,j);
        a2_x = (3 / (tf^2)) * (trajectory(1,j+1) - trajectory(1,j)) - ((2 / tf) * Velocity(1, j)) - ((1 / tf) * Velocity(1, j+1));
        a3_x = (-2 / (tf^3)) * (trajectory(1,j+1) - trajectory(1,j)) + ((1 / (tf^2)) * (Velocity(1, j+1) + Velocity(1, j)));

        % Polynom for y (Y-axis)
        ao_y = trajectory(2,j);
        a1_y = Velocity(2,j);
        a2_y = (3 / (tf^2)) * (trajectory(2,j+1) - trajectory(2,j)) - ((2 / tf) * Velocity(2, j)) - ((1 / tf) * Velocity(2, j+1));
        a3_y = (-2 / (tf^3)) * (trajectory(2,j+1) - trajectory(2,j)) + ((1 / (tf^2)) * (Velocity(2, j+1) + Velocity(2, j)));

        % Polynom for z (Z-axis)
        ao_z = trajectory(3,j);
        a1_z = Velocity(3,j);
        a2_z = (3 / (tf^2)) * (trajectory(3,j+1) - trajectory(3,j)) - ((2 / tf) * Velocity(3, j)) - ((1 / tf) * Velocity(3, j+1));
        a3_z = (-2 / (tf^3)) * (trajectory(3,j+1) - trajectory(3,j)) + ((1 / (tf^2)) * (Velocity(3, j+1) + Velocity(3, j)));
    else
        % Polynom for x (X-axis)
        ao_x = trajectory(1,j);
        a1_x = 0;
        a2_x = (3 / (tf^2)) * (trajectory(1,j+1) - trajectory(1,j));
        a3_x = (-2 / (tf^3)) * (trajectory(1,j+1) - trajectory(1,j));
        % Polynom for y (Y-axis)
        ao_y = trajectory(2,j);
        a1_y = 0;
        a2_y = (3 / (tf^2)) * (trajectory(2,j+1) - trajectory(2,j));
        a3_y = (-2 / (tf^3)) * (trajectory(2,j+1) - trajectory(2,j));
        % Polynom for z (Z-axis)
        ao_z = trajectory(3,j);
        a1_z = 0;
        a2_z = (3 / (tf^2)) * (trajectory(3,j+1) - trajectory(3,j));
        a3_z = (-2 / (tf^3)) * (trajectory(3,j+1) - trajectory(3,j));     
    end
    t = 0;
    while (t <= tf)
        
        polynom_x = ao_x + a1_x*t + a2_x*(t^2) + a3_x*(t^3);
        polynom_y = ao_y + a1_y*t + a2_y*(t^2) + a3_y*(t^3);
        polynom_z = ao_z + a1_z*t + a2_z*(t^2) + a3_z*(t^3);
               
        P = [polynom_x, polynom_y, polynom_z];

        inverseKinematics_solution = InverseKinematics(legsPositions_base,...
                                                       legsPositions_platform, P);

        legs_length = inverseKinematics_solution();
        
        forwardKinematics_solution = ForwardKinematics(legsPositions_base,...
                                                       legsPositions_platform,...
                                                       legs_length);
                                                     
        leg1_platform = forwardKinematics_solution + legsPositions_platform(:, 1);
        leg2_platform = forwardKinematics_solution + legsPositions_platform(:, 2);
        leg3_platform = forwardKinematics_solution + legsPositions_platform(:, 3);
        legsPositions_afterMoving = [leg1_platform, leg2_platform, leg3_platform];
                        
        DoThePlot(legsPositions_base, legsPositions_afterMoving, legs_length, P, trajectory);        
        
        t = t + 0.075;
        pause(0.05);
    end
end
