% test crane
clear
clc

theta = sym('theta',[1, 10],'real');
thetadot = sym('thetadot',[1, 10],'real');
thetaddot = sym('thetaddot',[1, 10],'real');
syms L1 L1dot L1ddot real;

wsl = 3;

g = 9.81;
for i = 1:5
    frames(i) = Frame('framenumber',i);
end

