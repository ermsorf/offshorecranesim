
function dispXddots = GetXddots(frames)

% % %%  Q-COORDINATES
% % %   The generalized coordinates
% % %   Thetaddot
    Qdd1 = frames(1).Qcoordinates(3);
    Qdd2 = frames(2).Qcoordinates(3);
    Qdd3 = frames(3).Qcoordinates(3);
    Qdd4 = frames(4).Qcoordinates(3);
    Qdd5 = frames(5).Qcoordinates(3);
    Qdd6 = frames(6).Qcoordinates(3);
    Qdd7 = frames(7).Qcoordinates(3);
    Qdd8 = frames(8).Qcoordinates(3);
    Qdd9 = frames(9).Qcoordinates(3);
    Qdd10 = frames(10).Qcoordinates(3);

%   Column for the generalized coordinates:
    QddMatrix = [Qdd1 ; Qdd2; Qdd3; Qdd4; Qdd5; Qdd6; Qdd7; Qdd8; Qdd9; Qdd10];


%% Q-COORDINATES
% Extract the generalized coordinates (Thetaddot) for frames 1 to 10
% Chat GPT was used to optimize the code

numFrames = 10; % Define number of frames
QddMatrix = sym(zeros(numFrames, 1)); % Preallocate for efficiency

for i = 1:numFrames
    QddMatrix(i) = frames(i).Qcoordinates(3);
end

QddMatrix;



%%  DEFINING Xddots UPDATE THIS, 
%   Extracting the x dots from array.
Bdot = frames(10).makeBdot(frames);

Xddots = Bdot*QddMatrix;

Xddot11 = Xddots(1)
Xddot21 = Xddots(2)
Xddot31 = Xddots(3)


Xddot12 = Xddots(7)
Xddot22 = Xddots(8)
Xddot32 = Xddots(9)

Xddot13 = Xddots(13)
Xddot23 = Xddots(14)
Xddot33 = Xddots(15)

Xddot14 = Xddots(19)
Xddot24 = Xddots(20)
Xddot34 = Xddots(21)

Xddot15 = Xddots(25)
Xddot25 = Xddots(26)
Xddot35 = Xddots(27)

Xddot16 = Xddots(31)
Xddot26 = Xddots(32)
Xddot36 = Xddots(33)

Xddot17 = Xddots(37)
Xddot27 = Xddots(38)
Xddot37 = Xddots(39)


Xddot18 = Xddots(43)
Xddot28 = Xddots(44)
Xddot38 = Xddots(45)

Xddot19 = Xddots(49)
Xddot29 = Xddots(50)
Xddot39 = Xddots(51)

Xddot110 = Xddots(55)
Xddot210 = Xddots(56)
Xddot310 = Xddots(57)


end