%This function plots the track layout.

import plot.Race.plotTrack
import track.*

track = Hockenheim2();

%width = 0.6; % For vehicles of size 3 x 1.5 m
width = 0.015; % For vehicles of size 0.066 x 0.03 m

figure
hold on
plot.private_plotTrack(track, width)