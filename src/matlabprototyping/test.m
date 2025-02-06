
theta = sym('theta',[1 5],'real')

for i = 1:4
    frames(i) = Frame('framenumber',i, 'rotationaxis', 3, 'rotationvar', theta(i));
end


ReactionForces(frames)
