import numpy as np
import sympy as sp
from sympy import symbols, Matrix, eye, cos, sin, diff, pprint, simplify, sstr


class Frame:
    def __init__(self, **kwargs):
        self.framenumber = None
        self.rotationaxis = None
        self.rotationvar = None
        self.cm2joint = np.array([0, 0, 0])
        self.joint2cm = np.array([0, 0, 0])
        self.Qcoordinates = None  # [[theta, thetadot, thetaddot], ...]
        self.initconditions = None
        self.Ematrix = None
        self.Edotmatrix = None
        self.Omatrix = None
        self.Bmatrix = None
        self.Bdotmatrix = None
        self.mass = None
        self.Jmatrix = None
        self.Mmatrix = None
        self.Dmatrix = None
        self.Fvec = None
        self.Tvec = None

        self.set_properties(**kwargs)

    def set_properties(self, **kwargs):
        for key, value in kwargs.items():
            if hasattr(self, key):
                setattr(self, key, value)
            else:
                raise AttributeError(f"Unknown property: {key}")
    @staticmethod
    def skew(vector):
        """ Returns the skew-symmetric matrix of a 3D vector """
        x, y, z = vector
        return np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])

    @staticmethod
    def unskew(matrix):
        """ Extracts a vector from a skew-symmetric matrix """
        return np.array([matrix[2, 1], matrix[0, 2], matrix[1, 0]])

    def makeEr(self):
        """ Create SE(3) rotation matrix based on axis and angle """
        theta = self.rotationvar
        axis =  self.rotationaxis
        if theta is None or axis is None or theta == 0 or axis == 0:
            return eye(4)
        Er = eye(4)
        c = cos(theta); s = sin(theta)
        if axis == 1:
            Er[1, 1], Er[1, 2] = c, -s
            Er[2, 1], Er[2, 2] = s, c
        elif axis == 2:
            Er[0, 0], Er[0, 2] = c, s
            Er[2, 0], Er[2, 2] = -s, c
        elif axis == 3:
            Er[0, 0], Er[0, 1] = c, -s
            Er[1, 0], Er[1, 1] = s, c
        return Er

    def makeEv(self, dispv):
        """ Create SE(3) translation matrix """
        Ev = eye(4)
        Ev[:3, 3] = Matrix(dispv)
        return Ev

    def makeE(self, framelist):
        """ Compute absolute transformation matrix """
        E = eye(4)
        for i in range(self.framenumber):
            frame = framelist[i]
            E = E @ frame.makeEv(frame.cm2joint) @ frame.makeEr() @ frame.makeEv(frame.joint2cm)
            E_simplified = E.applyfunc(simplify)
            frame.Ematrix = E_simplified
        return E_simplified

    def getQs(self, framelist):
        """ Aggregate Q coordinates from all frames """
        return Matrix.vstack(*[frame.Qcoordinates for frame in framelist])

    def makeEdot(self, framelist):
        """ Computes the time derivative of the transformation matrix E """
        if self.Ematrix is not None:
            E = self.Ematrix
        else:
            E = self.makeE(framelist)

        Q = self.getQs(framelist)  # Get Q values
        t = symbols('t', real=True)

        # Replace each theta with t * thetad
        prediff = E.subs({Q[i, 0]: t * Q[i, 1] for i in range(Q.shape[0])})

        # Differentiate with respect to t
        postdiff = diff(prediff, t)

        # Reverse substitution: replace t * thetad back with theta
        Edot = postdiff.subs({t * Q[i, 1]: Q[i, 0] for i in range(Q.shape[0])})

        self.Edotmatrix = Edot
        return Edot

    def makeO(self, framelist):
        """ Computes the O matrix """
        E = self.Ematrix if self.Ematrix is not None else self.makeE(framelist)
        Edot = self.Edotmatrix if self.Edotmatrix is not None else self.makeEdot(framelist)

        O = E.inv() @ Edot
        O_simplified = O.applyfunc(simplify)
        self.Omatrix = O_simplified
        return O_simplified

    def makeB(obj, framelist):
        Q = obj.getQs(framelist)
        numQs = Q.shape[0]
        B = sp.zeros(obj.framenumber * 6, numQs)

        for i in range(obj.framenumber):
            # Retrieve or compute Edot
            if framelist[i].Edotmatrix is not None:
                Edot = framelist[i].Edotmatrix
            else:
                Edot = framelist[i].makeEdot(framelist)
                framelist[i].Edotmatrix = Edot

            # Retrieve or compute O matrix
            if framelist[i].Omatrix is not None:
                O = framelist[i].Omatrix
            else:
                O = framelist[i].makeO(framelist)
                framelist[i].Omatrix = O

            posvec = Edot[:3, 3]
            Ovec = obj.unskew(O[:3, :3])

            cp_pos = [{} for _ in range(3)]
            tp_pos = [{} for _ in range(3)]
            cp_omega = [{} for _ in range(3)]
            tp_omega = [{} for _ in range(3)]

            for direction in range(3):
                cp_pos[direction], tp_pos[direction] = sp.poly(
                    posvec[direction]).as_expr().as_coefficients_dict(), list(Q[:, 1])
                cp_omega[direction], tp_omega[direction] = sp.poly(
                    Ovec[direction]).as_expr().as_coefficients_dict(), list(Q[:, 1])

            rowPos = slice(6 * i, 6 * i + 3)
            rowOmega = slice(6 * i + 3, 6 * i + 6)

            for q in range(numQs):
                for direction in range(3):
                    B[rowPos, q] = cp_pos[direction].get(Q[q, 1], 0)
                    B[rowOmega, q] = cp_omega[direction].get(Q[q, 1], 0)

        obj.Bmatrix = B
        return B

if __name__ == "__main__":
    g = 9.81  # Gravity constant
    theta = symbols('theta1 theta2 theta3 theta4 theta5 theta6 theta7 theta8 theta9 theta10')
    thetad = symbols('thetad1 thetad2 thetad3 thetad4 thetad5 thetad6 thetad7 thetad8 thetad9 thetad10')
    thetadd = symbols('thetadd1 thetadd2 thetadd3 thetadd4 thetadd5 thetadd6 thetadd7 thetadd8 thetadd9 thetadd10')

    frame1 = Frame(
        framenumber=1,
        rotationaxis=2,
        rotationvar=theta[0],
        Qcoordinates=Matrix([[theta[0], thetad[0], thetadd[0]]]),
        initconditions=[0, 0, 0],
        cm2joint=[0, 0, 0],
        joint2cm=[0.5, 0, 0],
        mass=10,
        Jmatrix=Matrix([[0, 0, 0], [0, 0, 0], [0, 0, 10 * 1 ** 2 / 12]]),
        Fvec=Matrix([[0, 0, -g * 10]]),
        Tvec=Matrix([[0, 0, 0]])
    )

    frame2 = Frame(
        framenumber= 2,
        rotationaxis= 2,
        rotationvar= theta[1],
        Qcoordinates=Matrix([[theta[1], thetad[1], thetadd[1]]]),
        initconditions=[0, 0, 0],
        cm2joint=[0.5, 0, 0],
        joint2cm=[0.5, 0, 0],
        mass=10,
        Jmatrix=Matrix([[0, 0, 0], [0, 0, 0], [0, 0, 10 * 1 ** 2 / 12]]),
        Fvec=Matrix([[0, 0, -g * 10]]),
        Tvec=Matrix([[0, 0, 0]])
    )

    frames = [frame1, frame2]

    pprint(sstr(frame2.makeE(frames)), num_columns=200)
    pprint(sstr(frame2.makeO(frames)), num_columns=200)