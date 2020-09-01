#include "jni.h"
#include "com_vendor_jni_VendorJNI.h"
#include "../../native/include/Eigen/Core"
#include "../../native/include/Eigen/Dense"
#include "../include/drake/math/discrete_algebraic_riccati_equation.h"
#include "../../native/include/Eigen/Eigenvalues"
#include "../../native/include/Eigen/QR"

JNIEXPORT jint JNICALL JNI_OnLoad(JavaVM* vm, void* reserved) {
    // Check to ensure the JNI version is valid

    JNIEnv* env;
    if (vm->GetEnv(reinterpret_cast<void**>(&env), JNI_VERSION_1_6) != JNI_OK)
        return JNI_ERR;

    // In here is also where you store things like class references
    // if they are ever needed

    return JNI_VERSION_1_6;
}

JNIEXPORT void JNICALL JNI_OnUnload(JavaVM* vm, void* reserved) {}

JNIEXPORT jdouble JNICALL Java_com_vendor_jni_VendorJNI_initialize
  (JNIEnv * env, jclass cls) {
    return 5;
}

JNIEXPORT jdoubleArray JNICALL Java_com_vendor_jni_VendorJNI_discreteAlgebraicRiccatiEquationJNI
  (JNIEnv * env, jclass cls, jdoubleArray A, jdoubleArray B, jdoubleArray Q, jdoubleArray R, jint states, jint inputs) {
    jdouble* nativeA = env->GetDoubleArrayElements(A, nullptr);
    jdouble* nativeB = env->GetDoubleArrayElements(B, nullptr);
    jdouble* nativeQ = env->GetDoubleArrayElements(Q, nullptr);
    jdouble* nativeR = env->GetDoubleArrayElements(R, nullptr);

    Eigen::Map<
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>
            Amat{nativeA, states, states};
    Eigen::Map<
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>
            Bmat{nativeB, states, inputs};
    Eigen::Map<
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>
            Qmat{nativeQ, states, states};
    Eigen::Map<
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>
            Rmat{nativeR, inputs, inputs};

    Eigen::MatrixXd result = drake::math::DiscreteAlgebraicRiccatiEquation(Amat, Bmat, Qmat, Rmat);

    env->ReleaseDoubleArrayElements(A, nativeA, 0);
    env->ReleaseDoubleArrayElements(B, nativeB, 0);
    env->ReleaseDoubleArrayElements(Q, nativeQ, 0);
    env->ReleaseDoubleArrayElements(R, nativeR, 0);

    jdoubleArray resultArray = env->NewDoubleArray(states*states);

    env->SetDoubleArrayRegion(resultArray, 0, states*states, result.data());

    return resultArray;
}
