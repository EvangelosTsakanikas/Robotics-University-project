function DoThePlot(legsPositions_base, legsPositions_afterMoving, legs_length, P, trajectoryPoints)

% %%%%%%%%%%%%%%% 3D plot
% % coordinates of legs (used during plot)
leg1_b = legsPositions_base(:, 1);
leg2_b = legsPositions_base(:, 2);
leg3_b = legsPositions_base(:, 3);

Xs_b = [leg1_b(1), leg2_b(1), leg3_b(1)];
Ys_b = [leg1_b(2), leg2_b(2), leg3_b(2)];
Zs_b = [leg1_b(3), leg2_b(3), leg3_b(3)];

leg1_p = legsPositions_afterMoving(:, 1);
leg2_p = legsPositions_afterMoving(:, 2);
leg3_p = legsPositions_afterMoving(:, 3);

Xs_p = [leg1_p(1), leg2_p(1), leg3_p(1)];
Ys_p = [leg1_p(2), leg2_p(2), leg3_p(2)];
Zs_p = [leg1_p(3), leg2_p(3), leg3_p(3)];

% plot functions
plot3([Xs_b(1) Xs_p(1)], [Ys_b(1) Ys_p(1)], [Zs_b(1) Zs_p(1)],...
      [Xs_b(2) Xs_p(2)], [Ys_b(2) Ys_p(2)], [Zs_b(2) Zs_p(2)],...
      [Xs_b(3) Xs_p(3)], [Ys_b(3) Ys_p(3)], [Zs_b(3) Zs_p(3)],...
      'Marker','.','LineStyle','-', 'Color', 'blue');

hold on
plot3([Xs_p(1) Xs_p(2)], [Ys_p(1) Ys_p(2)], [Zs_p(1) Zs_p(2)],...
      [Xs_p(1) Xs_p(3)], [Ys_p(1) Ys_p(3)], [Zs_p(1) Zs_p(3)],...
      [Xs_p(2) Xs_p(3)], [Ys_p(2) Ys_p(3)], [Zs_p(2) Zs_p(3)],...
      'Marker','.','LineStyle','-', 'LineWidth', 2, 'Color', 'red');
hold off

% useful tools for plot
grid on
axis([-3 3 -3 3 -3 3]);

xlabel('X-axis');
ylabel('Y-axis');
zlabel('Z-axis');

text(leg1_p(1), leg1_p(2), leg1_p(3)-0.01, ['  \leftarrow leg_1 length = ' num2str(legs_length(1))]);
text(leg2_p(1), leg2_p(2), leg2_p(3), ['  \leftarrow leg_2 length = ' num2str(legs_length(2))]);
text(leg3_p(1), leg3_p(2), leg3_p(3)+0.03, ['  \leftarrow leg_3 length = ' num2str(legs_length(3))]);
text(P(1), P(2), P(3), '.', 'FontWeight', 'bold', 'Fontsize', 16);
text(P(1), P(2), P(3)+2, ['  P('  num2str(P(1)) ', ' num2str(P(2)) ', ' num2str(P(3)) ')'], 'Fontsize', 12);

for j=1:size(trajectoryPoints, 2)
    text(trajectoryPoints(1,j), trajectoryPoints(2,j), trajectoryPoints(3,j),...
         '.', 'FontWeight', 'bold', 'Fontsize', 16, 'Color', 'green');
end
% %%%%%%%%%%%%%% end of 3D plot