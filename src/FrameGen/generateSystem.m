% clear
% clc

defineSystem


%% Create necessary matrices
noframes = length(frames);

Q = frames(noframes).getQs(frames)
B = frames(noframes).makeB(frames)
Bdot = frames(noframes).makeBdot(frames)
D = frames(noframes).makeD(frames);
M = frames(noframes).makeM(frames); 
F = frames(noframes).makeF(frames)
initCond = frames(noframes).getInitCond(frames)
Mstar = B' * M * B; 
Nstar = B' * (M*Bdot + D*M*B);


% Fstar = B' * F;
% eqs_of_motion = Mstar * Q(:,3) + Nstar*Q(:,2); eqs_of_motion =
% simplify(eqs_of_motion)



%% Create system struct for configexport()
info = struct('wiresegments', wiresegments);
system = struct( ...
    'info', info, ...
    'Qcoordinates', Q,...
    'initconditions', initCond,... 
    'Mstar', Mstar, ...
    'Nstar', Nstar, ...
    'B', B, ...
    'Bt', B',...
    'Bdot', Bdot,...
    'F', F); 

%{
reactions_eqs = load('Reactions_1_link_NEWTONS_12.05.2025_rev_2.mat');
rc = reactions_eqs.reactions;
rc = subs(rc, 'm2','16800');
rc = subs(rc, 'm3','250');
rc = subs(rc, 'm4','5000');
rc = subs(rc, 'mHook','500');
%}
reactions_eqs = load("Reaction_3_link_8_mai_2025.mat")
rc = reactions_eqs.reactions;
rc = subs(rc, 'Mload','5000');

system.reactions = rc

overwriteconfig = input(['Overwrite Config ' filename ' ? y/n: '], 's');
%%overwriteconfig == 'y'
if overwriteconfig == 'y'
    configexport(system, filename)
end
%}


