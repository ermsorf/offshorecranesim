classdef Frame < handle
    properties
        framenumber

        rotationaxis
        rotationvar

        cm2joint = [0, 0, 0]
        joint2cm = [0, 0, 0]

        Qcoordinates % Format: [theta, thetadot, thetaddot;...] for each coordinate
        
        initconditions

        relativeEmatrix
        Ematrix
        relativeEdotmatrix
        Edotmatrix
        relativeOmatrix
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
                if ~ismember(axis(i), [0, 1, 2, 3])
                    error('Axis must be 0 (No rotation), 1 (x-axis), 2 (y-axis), or 3 (z-axis).');
                end
                % Initialize identity SE3 matrix
                Ertemp = sym(eye(4));
                % Symbolic cos(theta) and sin(theta) for symbolic compatibility
                c = cos(theta(i));
                s = sin(theta(i));

                switch axis(i) % Rotation matrix depending on the chosen axis
                    case 0

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

        function Ev = makeEv(obj, dispv)
            % Create an SE3 transformation matrix using object properties
            % Uses joint2cm as the default displacement vector
            Ev = sym(eye(4));
            Ev(1:3, 4) = dispv(:)'; % Ensure column vector format
        end

        function relativeE = makeRelativeE(obj)
            relativeE = obj.makeEv(obj.cm2joint) * obj.makeEr() * obj.makeEv(obj.joint2cm);
            obj.relativeEmatrix = relativeE;
        end

        function E = makeE(obj, framelist)
            % Creates absolute transformation matrix E
            E = eye(4);  % Initialize as identity matrix
            for i = 1:obj.framenumber
                frame = framelist(i);
                E = E * frame.makeRelativeE();
                frame.Ematrix = E;
            end
            obj.Ematrix = E; 
        end

        function relativeEdot = makeRelativeEdot(obj)
            % Computes the time derivative of the transformation matrix E
            if ~isempty(obj.relativeEmatrix)
                E = obj.relativeEmatrix;
            else
                E = makeRelativeE(obj);
            end
            Q = obj.Qcoordinates;  % Get Q values
            Qsize = size(Q);
            syms t real
            % Vectorized substitution: replace each theta with t*thetadot
            prediff = subs(E, Q(:,1), t * Q(:,2));
            
            % Differentiate with respect to t
            postdiff = diff(prediff, t);
            
            % Reverse substitution: replace t*thetadot back with theta
            relativeEdot = subs(postdiff, t * Q(:,2), Q(:,1));
            
            obj.relativeEdotmatrix = relativeEdot;
        end


        function Edot = makeEdot(obj, framelist)
            % Computes the time derivative of the transformation matrix E
            if ~isempty(obj.Ematrix)
                E = obj.Ematrix;
            else
                E = makeE(obj, framelist);
            end
            if ~isempty(obj.Omatrix)
                O = obj.Omatrix;
            else
                O = obj.makeO(framelist);
            end
            Q = getQs(obj, framelist);  % Get Q values
            Qsize = size(Q);
            syms t real
            % Vectorized substitution: replace each theta with t*thetadot
            prediff = subs(E, Q(:,1), t * Q(:,2));
            
            % Differentiate with respect to t
            postdiff = diff(prediff, t);
            
            % Reverse substitution: replace t*thetadot back with theta
            Edot = subs(postdiff, t * Q(:,2), Q(:,1));
            
            obj.Edotmatrix = Edot;
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
        

        function relativeO = makeRelativeO(obj)
            %UNTITLED2 Summary of this function goes here
            if ~isempty(obj.relativeEmatrix)
                E = obj.relativeEmatrix;
            else
                E = makeRelativeE(obj);
            end
            if ~isempty(obj.relativeEdotmatrix)
                Edot = obj.Edotmatrix;
            else
                Edot = makeRelativeEdot(obj);
            end
            relativeO = inv(E) * Edot;
            obj.relativeOmatrix = relativeO;
        end % function makeO

        function O = makeO(obj, framelist)
            O = makeRelativeO(framelist(1));
            framelist(1).Omatrix = O;
            for i = 2:obj.framenumber
                frame = framelist(i);
                relE = makeRelativeE(frame);
                O = inv(relE) * O * relE * makeRelativeO(frame);
                frame.Omatrix = O;
            end

        end

        function B = makeB(obj, framelist)
            Q = getQs(obj, framelist);
            numQs = height(Q);
            Qvars = Q(:,2); % Extract Q variable symbols for faster indexing
            B = sym(zeros(obj.framenumber * 6, numQs));
        
            for i = 1:obj.framenumber
                % Retrieve or compute Edot
                if isempty(framelist(i).Edotmatrix)
                    framelist(i).Edotmatrix = framelist(i).makeEdot(framelist);
                end
                Edot = framelist(i).Edotmatrix;
        
                % Retrieve or compute O matrix
                if isempty(framelist(i).Omatrix)
                    framelist(i).Omatrix = framelist(i).makeO(framelist);
                end
                O = framelist(i).Omatrix;
        
                posvec = collect(Edot(1:3, 4), Qvars); % Collect terms
                Ovec   = collect(obj.unskew(O(1:3,1:3)), Qvars);
        
                % Extract coefficients for all directions at once
                [cp_pos, tp_pos] = arrayfun(@(dir) coeffs(posvec(dir), Qvars), 1:3, 'UniformOutput', false);
                [cp_omega, tp_omega] = arrayfun(@(dir) coeffs(Ovec(dir), Qvars), 1:3, 'UniformOutput', false);
        
                % Calculate row indices for this frame
                rowPos = 6*(i-1) + (1:3);
                rowOmega = rowPos + 3;
        
                % Populate B matrix efficiently
                for q = 1:numQs
                    qvar = Qvars(q);
                    for direction = 1:3
                        % For positions
                        idx = find(tp_pos{direction} == qvar, 1);
                        B(rowPos(direction), q) = ifelse(~isempty(idx), cp_pos{direction}(idx), sym(0));
                        
                        % For omegas
                        idx = find(tp_omega{direction} == qvar, 1);
                        B(rowOmega(direction), q) = ifelse(~isempty(idx), cp_omega{direction}(idx), sym(0));
                    end
                end
            end
        
            obj.Bmatrix = B;
        end


        function Bdot = makeBdot(obj, framelist)
        % Compute B and retrieve Q values
        B = obj.makeB(framelist);
        Q = obj.getQs(framelist);  % Assumes Q(:,1)=theta, Q(:,2)=thetadot, Q(:,3)=thetaddot
        syms t real
    
        % Vectorized substitution: replace Q(:,2) -> t*Q(:,3) and Q(:,1) -> t*Q(:,2)
        oldVars = [Q(:,2); Q(:,1)];
        newVals = [t * Q(:,3); t * Q(:,2)];
        prediff = subs(B, oldVars, newVals);
    
        % Differentiate with respect to t
        postdiff = diff(prediff, t);
    
        % Reverse substitution: replace t*Q(:,3) -> Q(:,2) and t*Q(:,2) -> Q(:,1)
        revOld = [t * Q(:,3); t * Q(:,2)];
        revNew = [Q(:,2); Q(:,1)];
        Bdot = subs(postdiff, revOld, revNew);

        % Optionally, assign to the object's property if desired:
        % obj.Bdotmatrix = Bdot;
    end
        
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

        function T = getTransformMat(obj, framelist)
            % Constructs T matrix 
            T = sym(zeros(obj.framenumber, 3));
            for i = 1:obj.framenumber
                if ~isempty(framelist(i).Ematrix)
                    Ematrix = framelist(i).Ematrix;
                else
                    Ematrix = framelist(i).makeE(framelist);
                    framelist(i).Ematrix = Ematrix;
                end % if Ematrix
                posvec = Ematrix(1:3,4);
                T(i,1:3) = posvec;
            end % for i
        end %function posExpr

        function rotations = exportRotations(obj, framelist)
            rotations = sym(zeros(obj.framenumber,2));
            for i = 1:obj.framenumber
                rotations(i,:) = [framelist(i).rotationaxis, framelist(i).rotationvar];
            end
        end

        function simplifiedMat = simplifyWithSymPy(obj, symMat)
            % Convert MATLAB symbolic matrix to a Python sympy matrix
            pyMat = py.sympy.Matrix(symMat(:).'); % Flatten, convert, then reshape later
        
            % Apply SymPy simplify function element-wise
            simplifiedPyMat = pyMat.applyfunc(py.sympy.simplify);
        
            % Convert back to MATLAB symbolic matrix
            simplifiedMat = reshape(sym(simplifiedPyMat), size(symMat));
        end

    end % methods
end % classdef