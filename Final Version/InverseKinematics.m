function inv_return = InverseKinematics(legsPositions_base,...
                                        legsPositions_platform, P)

p = [P(1); P(2); P(3)]; % Translation vector

a1 = legsPositions_base(:,1);
a2 = legsPositions_base(:,2);
a3 = legsPositions_base(:,3);

b1 = legsPositions_platform(:,1);
b2 = legsPositions_platform(:,2);
b3 = legsPositions_platform(:,3);

e1 = a1 - b1;
e2 = a2 - b2;
e3 = a3 - b3;

d1 = sqrt( transpose(p - e1) * (p - e1) );
d2 = sqrt( transpose(p - e2) * (p - e2) );
d3 = sqrt( transpose(p - e3) * (p - e3) );

legs_length = [d1; d2; d3];

% return the solution of Inverse Kinematics
inv_return = legs_length;