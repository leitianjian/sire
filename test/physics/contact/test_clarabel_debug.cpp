#include <iostream>
#include <Eigen/Dense>
#include <Eigen/Sparse>
using namespace Eigen;
using namespace std;
int main() {
    MatrixXd P = MatrixXd::Random(4,4);
    P = P * P.transpose();
    SparseMatrix<double> P_tt_sparse = MatrixXd(P.triangularView<Eigen::Upper>()).sparseView();
    P_tt_sparse.makeCompressed();
    cout << "rows: " << P_tt_sparse.rows() << " cols: " << P_tt_sparse.cols() << endl;
    cout << "nonzeros: " << P_tt_sparse.nonZeros() << endl;
    for (int k=0; k<P_tt_sparse.outerSize(); ++k) {
        for (SparseMatrix<double>::InnerIterator it(P_tt_sparse, k); it; ++it) {
            cout << "(" << it.row() << "," << it.col() << ") = " << it.value() << endl;
        }
    }
    return 0;
}
