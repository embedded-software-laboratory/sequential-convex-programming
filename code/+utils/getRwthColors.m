function colors = getRwthColors(percent)
%COLOR SELECTION: this function defines a selection of colors for the
%   plotting of multiple vehicles corresponding to the corporate design of
%   RWTH Aachen University. The colors are defined as RGB-triplets.
colors = zeros(8,3);

if percent == 100 % 100% colors
    colors(1,:) = [0 84 159]; % blue
    colors(2,:) = [246 168 0]; % orange
    colors(3,:) = [87 171 39]; % green
    colors(4,:) = [204 7 30]; % red
    colors(5,:) = [0 97 101]; % petrol
    colors(6,:) = [97 33 88]; % violet
    colors(7,:) = [87 171 39]; % turquoise
    colors(1,:) = [142 186 229]; % blue
elseif percent == 50  % 50% colors
    colors(1,:) = [142 186 229]; % blue
    colors(2,:) = [253 212 143]; % orange
    colors(3,:) = [184 214 152]; % green
    colors(4,:) = [230 150 121]; % red
    colors(5,:) = [125 164 167]; % petrol
    colors(6,:) = [168 133 158]; % violet
    colors(7,:) = [137 204 207]; % turquoise
    colors(1,:) = [142 186 229]; % blue
end

colors = colors / 255; % Get RGB triplet in values from 0 to 1
end