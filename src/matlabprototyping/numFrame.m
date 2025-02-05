classdef numFrame < handle
    %UNTITLED Summary of this class goes here
    %   Detailed explanation goes here

    properties
        framenumber

        rotationaxis
        anglepos
        anglevel
        angleacc

        pos
        vel
        acc

        cm2joint = [0,0,0]
        joint2cm = [0,0,0]

        Rrel
        Rabs

        Wrel
        Wabs

        Erel 
        Eabs

        Orel
        Oabs

        mass
        Jmatrix

        Fvec
        Tvec

        
    end

    methods
        function obj = numFrame(varargin)
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

        function V = unskew(matrix)
            % Unskew SO3 matrix to vector
            V = [matrix(3,2), matrix(1,3), matrix(2,1)];
        end

        function R = makeRrel(obj);
            axis = obj.rotationaxis;
            theta = obj.anglepos;
            R = eye(3);
            c = cos(theta);
            s = sin(theta);
            switch axis % Rotation matrix depending on the chosen axis
                case 1 
                    R(2,2) = c;  R(2,3) = -s;
                    R(3,2) = s;  R(3,3) = c;
                case 2 
                    R(1,1) = c;  R(1,3) = s;
                    R(3,1) = -s; R(3,3) = c;
                case 3 
                    R(1,1) = c;  R(1,2) = -s;
                    R(2,1) = s;  R(2,2) = c;
            end %switch axis
            obj.Rrel = R;
        end % function makeR

        function Rabs = makeRabs(obj, framelist);
            Rabs = eye(3);
            for n = 1:obj.framenumber;
                Rabs = Rabs * framelist(n).makeRrel();
                framelist(n).Rabs = Rabs;
            end
        end

        function Wrel = makeWrel(obj)
            axis = obj.rotationaxis;
            Wrel = eye(3);
            switch axis
                case 1
                    Wrel(2,3) = -obj.anglevel;
                    Wrel(3,2) = obj.anglevel;
                case 2
                    Wrel(3,1) = -obj.anglevel;
                    Wrel(1,3) = obj.anglevel;
                case 3
                    Wrel(1,2) = -obj.anglevel;
                    Wrel(2,1)  =obj.anglevel;
            end
            obj.Wrel = Wrel;
        end
        function Erel = makeErel(obj)
            Ecm2joint = eye(4); Ecm2joint(1:3,4) = obj.cm2joint';
            Er = eye(4); Er(1:3,1:3) = makeRrel(obj);
            Ejoint2cm = eye(4); Ejoint2cm(1:3,4) = obj.joint2cm;    
            Erel = Ecm2joint * Er * Ejoint2cm;
            obj.Erel = Erel;
        end

        function Eabs = makeEabs(obj, framelist)
            Eabs = eye(4);
            for n = 1:obj.framenumber
                Eabs = Eabs * framelist(n).makeErel();
                framelist(n).Eabs = Eabs;
            end
        end

        function Orel = makeOrel(obj)
            Orel = eye(4);
            obj.makeWrel(obj);
            Orel(1:3,1:3) = obj.Wrel;
            Orel(1:3,4) = transpose(obj.Rrel) * obj.vel;
        end

        function D = makeD(obj, framelist)
            error('NotImplemented')
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
            error('NotImplemented')
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

        function Fvector = makeF(obj,framelist)
            error('NotImplemented')
            frames = obj.framenumber;
            Fvector = zeros(frames*6,1);
            for i = 1:frames
                Fvector(i*6-5:i*6-3,1) = framelist(i).Fvec;
                Fvector(i*6-2:i*6,1) = framelist(i).Tvec;
            end
        end % Fvector
    end %methods
end %classdef