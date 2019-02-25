function forw_return = ForwardKinematics(legsPositions_base,...
                                         legsPositions_platform,...
                                         legs_length)
                                    
a1 = legsPositions_base(:,1);
a2 = legsPositions_base(:,2);
a3 = legsPositions_base(:,3);

b1 = legsPositions_platform(:,1);
b2 = legsPositions_platform(:,2);
b3 = legsPositions_platform(:,3);

e1 = a1 - b1;
e2 = a2 - b2;
e3 = a3 - b3;

d1 = legs_length(1);
d2 = legs_length(2);
d3 = legs_length(3);

k2 = ( transpose(e2)*e2 - transpose(e1)*e1 - d2^2 + d1^2 ) / 2;
k3 = ( transpose(e3)*e3 - transpose(e1)*e1 - d3^2 + d1^2 ) / 2;

delta_x = k2*(e3(2) - e1(2)) - k3*(e2(2) - e1(2)); 
delta_y = k3*(e2(1) - e1(1)) - k2*(e3(1) - e1(1));
delta = (e2(1) - e1(1))*(e3(2) - e1(2)) - (e3(1) - e1(1))*(e2(2) - e1(2));
delta_z = delta^2*(d1^2 - transpose(e1)*e1) - delta_x^2 - delta_y^2 + 2*delta*delta_x*e1(1) + 2*delta*delta_y*e1(2);

if (delta_z < 0)
    fprintf('! Cant calculate the movement \n');
    fprintf('No real solution found for these leg lengths: (%f, %f, %f)', legs_length(1),...
                                                                          legs_length(2),...
                                                                          legs_length(3));
    forw_return = -1;
else
    p_x = delta_x / delta;
    p_y = delta_y / delta;
    p_z = sqrt(delta_z) / delta;

    forw_return = [p_x; p_y; p_z];
end