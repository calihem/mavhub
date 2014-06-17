#ifndef _HUB_CONDITION_H
#define _HUB_CONDITION_H

#include <TooN/TooN.h>
#include <TooN/SVD.h>
#include <TooN/SymEigen.h>

namespace hub {

/**
 * \brief Calculates condition number of matrix.
 *
 * \tparam R Number of rows.
 * \tparam C Number of cols.
 * \tparam P Precision of matrix.
 * \tparam Norm Used matrix norm [-1,1,2].
 * \param matrix
 */
template <int R, int C, typename P, int Norm>
P condition(const TooN::Matrix<R,C,P> &matrix);

/**
 * \brief Calculates spectral norm of matrix.
 *
 * ||A||_2 := sqrt(max(eigenvalue(A^T * A)))
 *
 * \param[in] m Square matrix from which the norm is computed.
 */
template <int R, int C, typename P>
P norm_2(const TooN::Matrix<R,C,P> &m);

/************************************************
 * Implementation
 ***********************************************/
template <int R, int C, typename P>
P norm_2(const TooN::Matrix<R,C,P> &m) {

	TooN::SymEigen<C,P> sym_eigen(m.T()*m);
	const TooN::Vector<C,P> &eigenvalues = sym_eigen.get_evalues();
	const P &max_evalue = eigenvalues[ eigenvalues.size()-1 ];

	return sqrt(max_evalue);
}

template <int R, int C, typename P, int Norm>
P condition(const TooN::Matrix<R,C,P> &matrix) {

	TooN::SVD<R,C,P> svd(matrix);

	switch(Norm) {
		case -1: 
			return TooN::norm_inf(matrix) * TooN::norm_inf(svd.get_pinv());
		case 1:
			return TooN::norm_1(matrix) * TooN::norm_1(svd.get_pinv());
		case 2:
			return norm_2<R,C,P>(matrix) * norm_2<C,R,P>(svd.get_pinv());
	}

	// default case
	return TooN::norm_1(matrix) * TooN::norm_1(svd.get_pinv());
}

// Template specialisation
template <int R, int C, typename P>
inline P condition(const TooN::Matrix<R,C,P> &matrix) {
	return condition<R,C,P,2>(matrix);
}

} // namespace hub

#endif // _HUB_CONDITION_H

