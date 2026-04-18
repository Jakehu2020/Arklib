#include <stdexcept>
#include <cmath>
#include <cstring>

template <int Rows, int Cols>
class Mat {
public:
    double data[Rows * Cols];

    Mat() { std::memset(data, 0, sizeof(data)); }

    double& operator()(int r, int c) { return data[r * Cols + c]; }
    double operator()(int r, int c) const { return data[r * Cols + c]; }

    static Mat identity() {
        Mat m;
        for (int i = 0; i < Rows && i < Cols; ++i)
            m(i, i) = 1.0;
        return m;
    }

    Mat operator+(const Mat& rhs) const {
        Mat result;
        for (int i = 0; i < Rows * Cols; ++i)
            result.data[i] = data[i] + rhs.data[i];
        return result;
    }

    Mat operator-(const Mat& rhs) const {
        Mat result;
        for (int i = 0; i < Rows * Cols; ++i)
            result.data[i] = data[i] - rhs.data[i];
        return result;
    }

    Mat operator*(double s) const {
        Mat result;
        for (int i = 0; i < Rows * Cols; ++i)
            result.data[i] = data[i] * s;
        return result;
    }

    template <int K>
    Mat<Rows, K> operator*(const Mat<Cols, K>& rhs) const {
        Mat<Rows, K> result;
        for (int r = 0; r < Rows; ++r)
            for (int k = 0; k < K; ++k) {
                double sum = 0.0;
                for (int i = 0; i < Cols; ++i)
                    sum += (*this)(r, i) * rhs(i, k);
                result(r, k) = sum;
            }
        return result;
    }

    Mat<Cols, Rows> transpose() const {
        Mat<Cols, Rows> result;
        for (int r = 0; r < Rows; ++r)
            for (int c = 0; c < Cols; ++c)
                result(c, r) = (*this)(r, c);
        return result;
    }
};

// Gauss-Jordan inverse for square matrices
template <int N>
Mat<N, N> inverse(const Mat<N, N>& m) {
    double aug[N][N * 2]; // Augmented matrix [m | I]
    for (int r = 0; r < N; ++r) {
        for (int c = 0; c < N; ++c)
            aug[r][c] = m(r, c);
        for (int c = 0; c < N; ++c)
            aug[r][N + c] = (r == c) ? 1.0 : 0.0;
    }

    for (int col = 0; col < N; ++col) {
        int pivot = col;
        for (int r = col + 1; r < N; ++r)
            if (std::fabs(aug[r][col]) > std::fabs(aug[pivot][col]))
                pivot = r;

        if (pivot != col)
            for (int c = 0; c < N * 2; ++c)
                std::swap(aug[col][c], aug[pivot][c]);

        double div = aug[col][col];
        for (int c = 0; c < N * 2; ++c)
            aug[col][c] /= div;

        for (int r = 0; r < N; ++r) {
            if (r == col) continue;
            double factor = aug[r][col];
            for (int c = 0; c < N * 2; ++c)
                aug[r][c] -= factor * aug[col][c];
        }
    }

    Mat<N, N> result;
    for (int r = 0; r < N; ++r)
        for (int c = 0; c < N; ++c)
            result(r, c) = aug[r][N + c];
    return result;
}

// Kalman Filter Equations (reference):
// State equation: x_k = F * x_{k-1} + B * u_k + w; w ~ N(0, Q)
// Measurement eq: z_k = H * x_k + v; v ~ N(0, R)

// n: state dimension, m: measurement dimensions, c: control dimensions (default 1; set B to zero mat if unused)
template <int n, int m, int c = 1>
class KalmanFilter {
public:
    typedef Mat<n, 1> StateVec;
    typedef Mat<n, n> StateMat;
    typedef Mat<m, 1> MeasVec;
    typedef Mat<m, m> MeasMat;
    typedef Mat<m, n> ObsMat;
    typedef Mat<n, m> GainMat;
    typedef Mat<c, 1> ControlVec;
    typedef Mat<n, c> ControlMat;

    /**
     * @param F State transition matrix (n x n)
     * @param H Observation matrix (m x n)
     * @param Q Process noise covariance (n x n)
     * @param R Measurement noise covariance (m x m)
     * @param B Control-input matrix (n x c), defaults to zero
     */
    KalmanFilter(const StateMat& F, // State transition matrix
                 const ObsMat& H, // Observation matrix
                 const StateMat& Q, // Process noise covariance
                 const MeasMat& R, // Measurement noise covariance
                 const ControlMat& B = ControlMat()) // Control-input matrix
        : F_(F), H_(H), Q_(Q), R_(R), B_(B),
          x_(), P_(StateMat::identity()),
          initialized_(false) {}

    void init(const StateVec& x0,
              const StateMat& P0 = StateMat::identity()) {
        x_ = x0;
        P_ = P0;
        initialized_ = true;
    }

    void predict(const ControlVec& u = ControlVec()) {
        x_ = F_ * x_ + B_ * u;
        P_ = F_ * P_ * F_.transpose() + Q_;
    }

    void update(const MeasVec& z) {
        MeasVec y = z - H_ * x_; // innovation
        MeasMat S = H_ * P_ * H_.transpose() + R_; // innovation covariance
        GainMat K = P_ * H_.transpose() * inverse(S); // Kalman gain

        x_ = x_ + K * y;
        P_ = (StateMat::identity() - K * H_) * P_;
    }

    void step(const MeasVec& z, const ControlVec& u = ControlVec()) {
        predict(u);
        update(z);
    }

    const StateVec& state() const { return x_; }
    const StateMat& covariance() const { return P_; }
    bool initialized() const { return initialized_; }

    void setF(const StateMat& F) { F_ = F; }
    void setH(const ObsMat& H) { H_ = H; }
    void setQ(const StateMat& Q) { Q_ = Q; }
    void setR(const MeasMat& R) { R_ = R; }
    void setB(const ControlMat& B) { B_ = B; }

private:
    StateMat F_;
    ObsMat H_;
    StateMat Q_;
    MeasMat R_;
    ControlMat B_;

    StateVec x_;
    StateMat P_;

    bool initialized_;
};