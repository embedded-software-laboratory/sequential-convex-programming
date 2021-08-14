function colors = getRwthColors(percent)
%COLOR SELECTION: this function defines a selection of colors for the
%   plotting of multiple vehicles corresponding to the corporate design of
%   RWTH Aachen University. The colors are defined as RGB-triplets.
colors = zeros(8,3);

if percent == 100 % 100% colors
    colors(1,:) = [246 168 0]; % orange
    colors(2,:) = [87 171 39]; % green
    colors(3,:) = [204 7 30]; % red
    colors(4,:) = [0 97 101]; % petrol
    colors(5,:) = [97 33 88]; % violet
    colors(6,:) = [87 171 39]; % turquoise
    colors(7,:) = [0 84 159]; % blue
elseif percent == 50  % 50% colors
    colors(1,:) = [253 212 143]; % orange
    colors(2,:) = [184 214 152]; % green
    colors(3,:) = [230 150 121]; % red
    colors(4,:) = [125 164 167]; % petrol
    colors(5,:) = [168 133 158]; % violet
    colors(6,:) = [137 204 207]; % turquoise
    colors(7,:) = [142 186 229]; % blue
end

colors = colors / 255; % Get RGB triplet in values from 0 to 1
end