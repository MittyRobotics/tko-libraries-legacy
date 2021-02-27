package com.github.mittyrobotics.motion.observers;

import org.apache.commons.math3.linear.Array2DRowRealMatrix;
import org.apache.commons.math3.linear.ArrayRealVector;
import org.apache.commons.math3.linear.CholeskyDecomposition;
import org.apache.commons.math3.linear.RealMatrix;
import org.ejml.data.DMatrixRMaj;
import org.ejml.data.MatrixType;
import org.ejml.dense.row.factory.DecompositionFactory_DDRM;
import org.ejml.dense.row.factory.LinearSolverFactory_DDRM;
import org.ejml.dense.row.linsol.chol.LinearSolverChol_DDRM;
import org.ejml.interfaces.decomposition.CholeskyDecomposition_F64;
import org.ejml.interfaces.linsol.LinearSolverDense;
import org.ejml.simple.SimpleMatrix;

public class KalmanFilter {
    private final SimpleMatrix A; //State-transition matrix, applied to current state vector to get next state vector
    private final SimpleMatrix AT; //A transposed
    private final SimpleMatrix B; //Control matrix, applied to control vector u to get change in state
    private final SimpleMatrix H; //Measurement matrix, influences kalman gain
    private final SimpleMatrix HT; //H transposed
    private SimpleMatrix xHat; //Estimated state
    private final SimpleMatrix Q; //Process covariance matrix, estimated accuracy of prediction
    private final SimpleMatrix R; //Measurement covariance matrix, estimated accuracy of measurements
    private SimpleMatrix P; //Estimated error covariance matrix

    /**
     * Constructs a Kalman Filter
     * @param A State-transition matrix, applied to current state vector to get next state vector
     * @param B Control matrix, applied to control vector u to get change in state
     * @param H Measurement matrix, influences kalman gain
     * @param Q Process covariance matrix, estimated accuracy of prediction
     * @param R Measurement covariance matrix, estimated accuracy of measurements
     */
    public KalmanFilter(SimpleMatrix A, SimpleMatrix B, SimpleMatrix H, SimpleMatrix Q, SimpleMatrix R){
        this(A, B, H, new SimpleMatrix(A.numRows(), 1), Q, R);
    }

    /**
     * Constructs a Kalman Filter
     * @param A State-transition matrix, applied to current state vector to get next state vector
     * @param B Control matrix, applied to control vector u to get change in state
     * @param H Measurement matrix, influences kalman gain
     * @param xHat Initial state vector
     * @param Q Process covariance matrix, estimated accuracy of prediction
     * @param R Measurement covariance matrix, estimated accuracy of measurements
     */
    public KalmanFilter(SimpleMatrix A, SimpleMatrix B, SimpleMatrix H, SimpleMatrix xHat, SimpleMatrix Q, SimpleMatrix R){
        this(A, B, H, xHat, Q, R, new SimpleMatrix(A.numRows(), A.numCols()));
    }

    /**
     * Constructs a Kalman Filter
     * @param A State-transition matrix, applied to current state vector to get next state vector
     * @param B Control matrix, applied to control vector u to get change in state
     * @param H Measurement matrix, influences kalman gain
     * @param xHat Initial state vector
     * @param Q Process covariance matrix, estimated accuracy of prediction
     * @param R Measurement covariance matrix, estimated accuracy of measurements
     * @param P Estimated error covariance matrix
     */
    public KalmanFilter(SimpleMatrix A, SimpleMatrix B, SimpleMatrix H, SimpleMatrix xHat, SimpleMatrix Q, SimpleMatrix R, SimpleMatrix P){
        this.A = A;
        this.AT = A.transpose();
        this.B = B;
        this.H = H;
        this.HT = H.transpose();
        this.xHat = xHat;
        this.Q = Q;
        this.R = R;
        this.P = P;
    }

    /**
     * Predicts next state
     * @param u Control input matrix
     */
    public void predict(SimpleMatrix u){
        xHat = A.mult(xHat).plus(B.mult(u));
        P = A.mult(P).mult(AT).plus(Q);
    }

    /**
     * Corrects state
     * @param z measurement matrix
     */
    public void correct(SimpleMatrix z){
        SimpleMatrix S = H.mult(P).mult(HT).plus(R);
        SimpleMatrix y = z.minus(H.mult(xHat));

        LinearSolverDense<DMatrixRMaj> chol = LinearSolverFactory_DDRM.chol(S.numRows());
        chol.setA(S.getMatrix());
        SimpleMatrix output = new SimpleMatrix(S.numRows(), S.numCols());
        chol.solve(H.mult(P.transpose()).getMatrix(), output.getMatrix());

        SimpleMatrix K = output.transpose();
        xHat = xHat.plus(K.mult(y));
        P = SimpleMatrix.identity(K.numRows()).minus(K.mult(H)).mult(P);
    }

    public void setxHat(SimpleMatrix xHat){
        this.xHat = xHat;
    }

    public KalmanFilter copyOf(){
        return new KalmanFilter(A, B, H, xHat, Q, R, P);
    }

    public SimpleMatrix getA() {
        return A;
    }

    public SimpleMatrix getB() {
        return B;
    }

    public SimpleMatrix getH() {
        return H;
    }

    public SimpleMatrix getxHat() {
        return xHat;
    }

    public SimpleMatrix getQ() {
        return Q;
    }

    public SimpleMatrix getR() {
        return R;
    }

    public SimpleMatrix getP() {
        return P;
    }
}
