classdef Frame < handle
    properties
        framenumber

        rotationaxis
        rotationvar

        cm2joint = [0, 0, 0]
        joint2cm = [0, 0, 0]

        Qcoordinates % Format: [theta, thetadot, thetaddot;...] for each coordinate
        
        initconditions

        Ematrix
        Edotmatrix
        Omatrix
        Bmatrix
        Bdotmatrix

        mass
        Jmatrix
        Mmatrix

        Dmatrix
        Fvec
        Tvec

    end

    methods
        function obj = Frame(varargin)
            % Constructor accepting Name-Value pairs
            if nargin > 0
                obj.setProperties(varargin{:});
            end
        end
        
        function setProperties(obj, varargin)
            % Set multiple properties using Name-Value pairs
            if mod(length(varargin), 2) ~= 0
                error('Arguments must be Name-Value pairs.');
            end
            
            for i = 1:2:length(varargin)
                propName = varargin{i};
                propValue = varargin{i+1};
                
                if isprop(obj, propName)
                    obj.(propName) = propValue;
                else
                    error('Unknown property: %s', propName);
                end
            end
        end % function setProperties

        function displayInfo(obj)
            % Display frame info
            fprintf('Frame %d\n', obj.framenumber);
            fprintf('Rotation Axis: [%s]\n', num2str(obj.rotationaxis));
            fprintf('Mass: %f\n', obj.mass);
            fprintf('Q Coordinates: %s\n', mat2str(obj.Qcoordinates));
        end

        function skewmat = skew(vector)
        % Skew a 1x3 or 3x1 vector
        %   Detailed explanation goes here
            if length(vector) == 1 
                vector = vector' ; 
            end
            skewmat = zeros(3,3);
            skewmat(2,3) = -vector(1); skewmat(3,2) =  vector(1);
            skewmat(1,3) =  vector(2); skewmat(3,1) = -vector(2);
            skewmat(1,2) = -vector(3); skewmat(2,1) =  vector(3);
        end
        function V = unskew(~, matrix)
            % Unskew SO3 matrix to vector
            V = [matrix(3,2); matrix(1,3); matrix(2,1)];
        end
        % Make E's
        function Ertemp = makeEr(obj)
            % Create an SE3 transformation matrix for a given rotation axis
            axis = obj.rotationaxis;
            theta = obj.rotationvar;
            Er = sym(eye(4));

            for i = 1:length(axis)
                % Validate input
                if ~ismember(axis(i), [1, 2, 3])
                    error('Axis must be 1 (x-axis), 2 (y-axis), or 3 (z-axis).');
                end
                % Initialize identity SE3 matrix
                Ertemp = sym(eye(4));
                % Symbolic cos(theta) and sin(theta) for symbolic compatibility
                c = cos(theta(i));
                s = sin(theta(i));

                switch axis(i) % Rotation matrix depending on the chosen axis
                    case 1 
                        Ertemp(2,2) = c;  Ertemp(2,3) = -s;
                        Ertemp(3,2) = s;  Ertemp(3,3) = c;
                    case 2 
                        Ertemp(1,1) = c;  Ertemp(1,3) = s;
                        Ertemp(3,1) = -s; Ertemp(3,3) = c;
                    case 3 
                        Ertemp(1,1) = c;  Ertemp(1,2) = -s;
                        Ertemp(2,1) = s;  Ertemp(2,2) = c;
                end
                Er = Er*Ertemp;
            end

        end

        function Ev = makeEv(obj)
            % Create an SE3 transformation matrix using object properties
            % Uses joint2cm as the default displacement vector
            Ev = sym(eye(4));
            Ev(1:3, 4) = obj.joint2cm(:); % Ensure column vector format
        end
        
        function E = makeE(obj, framelist)
            % Creates absolute transformation matrix E
            E = eye(4);  % Initialize as identity matrix
            for i = 1:obj.framenumber
                frame = framelist(i);
                E = E * frame.makeEv() * frame.makeEr() * frame.makeEv();
                frame.Ematrix = E;
            end
            obj.Ematrix = simplify(E);  % Simplify the resulting matrix
        end


        function Edot = makeEdot(obj, framelist)
            % Computes the time derivative of the transformation matrix E
            if ~isempty(obj.Ematrix)
                E = obj.Ematrix;
            else
                E = makeE(obj, framelist);
            end
            Q = getQs(obj, framelist);  % Get Q values
            Qsize = size(Q);
            syms t real
            prediff = E;
            for i = 1:Qsize(1)  % Replace Q with differentiable variables
                prediff = subs(prediff, Q(i,1), t * Q(i,2));
            end
        
            postdiff = diff(prediff, t);  % Differentiate with respect to t
            Edot = postdiff;
            
            for i = 1:Qsize(1)  % Replace t*Q(2,i) with Q(1,i)
                Edot = subs(Edot, t*Q(i,2), Q(i,1));
            end
            obj.Edotmatrix = simplify(Edot);  % Simplify result
        end

        function Q_combined = getQs(obj, framelist)
            % Combines time-dependent Q coordinates from multiple frames
            Q_combined = sym([]);
            for i = 1:(obj.framenumber)
                Q_combined = [Q_combined; framelist(i).Qcoordinates];
            end
        end
        function initCond = getInitCond(obj, framelist)
            % Combines time-dependent Q coordinates from multiple frames
            initCond = sym([]);
            for i = 1:(obj.framenumber)
                initCond = [initCond; framelist(i).initconditions];
            end
        end


        function O = makeO(obj, framelist)
            %UNTITLED2 Summary of this function goes here
            if ~isempty(obj.Ematrix)
                E = obj.Ematrix;
            else
                E = makeE(obj, framelist);
            end
            if ~isempty(obj.Edotmatrix)
                Edot = obj.Edotmatrix;
            else
                Edot = makeEdot(obj, framelist);
            end
            O = inv(E) * Edot;
            obj.Omatrix = simplify(O);
        end % function makeO

        function B = makeB(obj, framelist)
            % Constructs B matrix 
            Q = getQs(obj, framelist);    % Initialize B matrix
            B = sym(zeros(obj.framenumber*6, height(Q)));
            for i = 1:obj.framenumber
                if ~isempty(framelist(i).Edotmatrix)
                    Edot = framelist(i).Edotmatrix;
                else
                    Edot = framelist(i).makeEdot(framelist);
                    framelist(i).Edotmatrix = Edot;
                end % if Edot
                if ~isempty(framelist(i).Omatrix)
                    O = obj.Omatrix;
                else
                    O = framelist(i).makeO(framelist);
                    framelist(i).Omatrix = O;
                end % if O
                posvec = Edot(1:3,4);
                Ovec = obj.unskew(O(1:3,1:3)); 
                for q = 1:height(Q) %% For every q in q list
                    for direction = 1:3 %% For every direction
                        %% Positions
                        [cp, tp] = coeffs(posvec(direction), Q(q,2)); % Get coefficients and terms
                        if isempty(cp) || isempty(tp) % Handle empty symbolic vectors
                            B((6*i-6+direction), q) = sym(0);
                        else
                            idx = find(tp == Q(q,2), 1); % Find the coefficient of Q(q)^1
                            if ~isempty(idx)
                                B((6*i-6+direction), q) = cp(idx);
                            else
                                B((6*i-6+direction), q) = sym(0); % Store 0 if Q(q) is absent
                            end
                        end % if coeffs
                        %% Omegas
                        [cr, tr] = coeffs(Ovec(direction), Q(q,2)); % Get coefficients and terms
                        if isempty(cr) || isempty(tr) % Handle empty symbolic vectors
                            B((6*i-3+direction), q) = sym(0);
                        else
                            idx = find(tr == Q(q,2), 1); % Find the coefficient of Q(q)^1
                            if ~isempty(idx)
                                B((6*i-3+direction), q) = cr(idx);
                            else
                                B((6*i-3+direction), q) = sym(0); % Store 0 if Q(q) is absent
                            end
                        end % if coeffs
                    end % for direction
                end % for q         
            end % for i
            B = simplify(B);
            obj.Bmatrix = B;
        end %function B

        function Bdot = makeBdot(obj,framelist)
            % Get Bdot
                B = makeB(obj,framelist);
            Q = getQs(obj, framelist);  % Get Q values
            syms t real
            prediff = B;
            for i = 1:height(Q)  % Replace Q with differentiable variables
                prediff = subs(prediff, Q(i,2), t*Q(i,3));
                prediff = subs(prediff, Q(i,1), t*Q(i,2));
            end
            postdiff = diff(prediff, t);  % Differentiate with respect to t
            Bdot = postdiff;
            for i = 1:height(Q)  % Replace t*Q(2,i) with Q(1,i)
                Bdot = subs(Bdot, t*Q(i,3), Q(i,2));
                Bdot = subs(Bdot, t*Q(i,2), Q(i,1));
            end
            Bdot = simplify(Bdot);  % Simplify result
            obj.Bdotmatrix
        end % function makeBdot
        
        function D = makeD(obj, framelist)
            % Construct D matrix
            D = sym(zeros(obj.framenumber*6,obj.framenumber*6));
            
            for i = 1:obj.framenumber
                if ~isempty(framelist(i).Omatrix)
                    O = framelist(i).Omatrix;
                else
                    O = makeO(obj,framelist);
                end
                w = O(1:3,1:3);
                index1 = i*6-2  ;
                index2 = i*6 ;
                D(index1:index2,index1:index2) = w;
            end
        end % function makeD
        
        function Mmatrix = makeM(obj, framelist)
            % Makes M matrix
            size = obj.framenumber *6;
            Mmatrix = sym(zeros(size,size));
            
            for i = 1:obj.framenumber
                if isempty(framelist(i).mass) | isempty(framelist(i).Jmatrix)
                    error('Missing mass or J for frame %i',framelist(i).framenumber)
                end
                massIdentity = framelist(i).mass .* eye(3);
                Mmatrix(i*6-5:i*6-3,i*6-5:i*6-3) = massIdentity;
                Mmatrix(i*6-2:i*6,i*6-2:i*6) = framelist(i).Jmatrix;
            end
        end % function makeM

        function F = makeF(obj, framelist)
            F = sym(zeros(obj.framenumber*6,1));
            for i = 1:obj.framenumber
                F(6*i-5:6*i-3,1) = framelist(i).Fvec';
                F(6*i-2:6*i,1) = framelist(i).Tvec';
            end
            
        end
        
    end % methods
end % classdef