function [R,Rdot] = makeR(frame)
% Generate relative R matrices from frame struct
    axis = frame.rotationaxis ;
    var  = frame.rotationvar ;
    vardot = frame.rotationvardot ;
    
    c = cos(var); 
    s = sin(var);
    
    switch axis
        case 1
           R = [1, 0, 0; 0, c, -s; 0, s, c]
           Rdot = [0,0,0; 0,0,-vardot; 0,vardot,0]

        case 2
            R = [c,0,s; 0,1,0; -s,0,c]
            Rdot = [0,0,vardot; 0,0,0; -vardot,0,0]

        case 3
            R = [c,-s,0; s,c,0; 0,0,1]
            Rdot = [0,-vardot,0; vardot,0,0; 0,0,0]
    end
end