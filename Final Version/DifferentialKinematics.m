function diff_return = DifferentialKinematics(legsPositions_base,...
                                              legsPositions_platform, legs_length, q_dot)
                                          
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

p = ForwardKinematics(legsPositions_base,...
                      legsPositions_platform, legs_length);

if (p ~= -1)
    s1 = (p - e1) / d1;
    s2 = (p - e2) / d2;
    s3 = (p - e3) / d3;

    J = [transpose(s1); transpose(s2); transpose(s3)];

    if (det(J) ~= 0)
        inverted_J = inv(J);
        Vp = inverted_J * q_dot;
        diff_return = Vp;
    else
        diff_return = -1;
        fprintf('Cannot invert Jacobian matrix \n');
    end    
else
    diff_return = -1;
    fprintf('\n\n');
end


                  


