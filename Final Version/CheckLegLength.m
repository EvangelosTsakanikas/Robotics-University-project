function checkLegLength =  CheckLegLength(legs_length)

maxLeg_length = 4;
minLeg_length = 1;

if (max(legs_length) > maxLeg_length)
    fprintf('\n');
    fprintf('! Error \n');
    fprintf('the length of the legs exceeds the max leg length (%0.2f) \n', maxLeg_length);
    fprintf('leg lengths: (%f, %f, %f)', legs_length(1),...
                                         legs_length(2),...
                                         legs_length(3));
    fprintf('\n\n');
    checkLegLength = 0;
elseif (min(legs_length) < minLeg_length)
    fprintf('\n');
    fprintf('! Error \n');
    fprintf('the length of the legs is less than the min leg length (%f) \n', minLeg_length);
    fprintf('leg lengths: (%f, %f, %f)', legs_length(1),...
                                         legs_length(2),...
                                         legs_length(3));
    fprintf('\n\n');
    checkLegLength = 0;
else
    checkLegLength = 1;
end