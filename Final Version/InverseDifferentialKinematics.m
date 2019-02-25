function inv_diff_return = InverseDifferentialKinematics(legsPositions_base,...
                                                         legsPositions_platform,...
                                                         p, Vp)

a1 = legsPositions_base(:,1);
a2 = legsPositions_base(:,2);
a3 = legsPositions_base(:,3);

b1 = legsPositions_platform(:,1);
b2 = legsPositions_platform(:,2);
b3 = legsPositions_platform(:,3);

e1 = a1 - b1;
e2 = a2 - b2;
e3 = a3 - b3;

inverseKinematics_solution = InverseKinematics(legsPositions_base,...
                                               legsPositions_platform, p);
                                           
d1 = inverseKinematics_solution(1);
d2 = inverseKinematics_solution(2);
d3 = inverseKinematics_solution(3);

legs_length = [d1; d2; d3];
if (CheckLegLength(legs_length) == 1)
    s1 = (p - e1) / d1;
    s2 = (p - e2) / d2;
    s3 = (p - e3) / d3;

    J = [transpose(s1); transpose(s2); transpose(s3)];

    q_dot = J * Vp;

    inv_diff_return = q_dot;
else
    inv_diff_return = -1;
end