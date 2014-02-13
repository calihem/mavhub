#include "bundleadjust.h"

#if defined HAVE_OPENCV2 && CV_MINOR_VERSION >= 2

#include "lib/slam/camera.h"
#include <sba/sba_chkjac.h>
#include <sba/compiler.h>

#ifdef __cplusplus
extern "C" {
#endif

#define emalloc(sz)       emalloc_(__FILE__, __LINE__, sz)
#define FABS(x)           (((x)>=0)? (x) : -(x))
#define COLUMN_MAJOR      1
#define MAT_STORAGE       COLUMN_MAJOR
#define SBA_EPSILON       1E-12
#define SBA_EPSILON_SQ    ( (SBA_EPSILON)*(SBA_EPSILON) )
#define SBA_ONE_THIRD     0.3333333334 /* 1.0/3.0 */

typedef int (*PLS)(double *A, double *B, double *x, int m, int iscolmaj);

struct fdj_data_x_ {
	void (*func)(double *p, struct sba_crsm *idxij, int *rcidxs, int *rcsubs, double *hx, void *adata); /* function to differentiate */
	int cnp, pnp, mnp;
	int *func_rcidxs,
	*func_rcsubs;
	double *hx, *hxx;
	void *adata;
};

inline static void *emalloc_(const char *file, int line, size_t sz) {
	void *ptr=(void *)malloc(sz);
	if(ptr==NULL) {
		fprintf(stderr, "SBA: memory allocation request for %u bytes failed in file %s, line %d, exiting", sz, file, line);
		exit(1);
	}
	return ptr;
}

inline static void _dblzero(register double *arr, register int count) {
	while(--count>=0)
	*arr++=0.0;
}

extern void sba_fdjac_x(double *p, struct sba_crsm *idxij, int *rcidxs, int *rcsubs, double *jac, void   *dat);
extern double nrmL2xmy(double *const e, const double *const x, const double *const y, const int n);
extern double nrmCxmy(double *const e, const double *const x, const double *const y, const double *const W, const int mnp, const int nvis);
extern double sba_mean_repr_error(int n, int mnp, double *x, double *hx, struct sba_crsm *idxij, int *rcidxs, int *rcsubs);
extern void sba_print_inf(double *hx, int nimgs, int mnp, struct sba_crsm *idxij, int *rcidxs, int *rcsubs);

#ifdef __cplusplus
}
#endif

namespace hub {
namespace slam {

void sba_ideal_pinhole_model(double *p, struct sba_crsm *idxij, int *rcidxs, int *rcsubs, double *hx, void *adata) {
	const int num_cols = idxij->nc;
	const double *p_points = p+num_cols*num_params_per_cam;

	for(int j=0; j<num_cols; ++j) {
		// get j-th camera parameters
		const double *p_pose = p + j*num_params_per_cam;

		// find number of nonzero hx_ij, i=0...n-1
		const int num_nonzero = sba_crsm_col_elmidxs(idxij, j, rcidxs, rcsubs);
		for(int i=0; i<num_nonzero; ++i) {
			const double *p_point = p_points + rcsubs[i]*num_params_per_point;
			double *p_measurement = hx + idxij->val[rcidxs[i]]*num_params_per_measuremnt; // p_measurement = hx_ij

			ideal_pinhole_model_quatvec<double>(p_point, p_pose, p_measurement);
		}
	}
}

void sba_pinhole_model(double *p, struct sba_crsm *idxij, int *rcidxs, int *rcsubs, double *hx, void *adata) {
	const sba_model_data_t<double> *sba_data = static_cast< sba_model_data_t<double>* >(adata);
	if(!sba_data) return;

	const cv::Mat &camera_matrix = sba_data->camera_matrix;

	const int num_cols = idxij->nc;
	const double *p_points = p+num_cols*num_params_per_cam;

	for(int j=0; j<num_cols; ++j) {
		// get j-th camera parameters
		const double *p_pose = p + j*num_params_per_cam;

		// find number of nonzero hx_ij, i=0...n-1 
		const int num_nonzero = sba_crsm_col_elmidxs(idxij, j, rcidxs, rcsubs);
		for(int i=0; i<num_nonzero; ++i) {
			const double *p_point = p_points + rcsubs[i]*num_params_per_point;
			double *p_measurement = hx + idxij->val[rcidxs[i]]*num_params_per_measuremnt; // p_measurement = hx_ij
			
			pinhole_model_quatvec<double>(p_point, p_pose, camera_matrix, p_measurement);
		}
	}
}

void sba_ideal_pinhole_model_jac(double *p, struct sba_crsm *idxij, int *rcidxs, int *rcsubs, double *jac, void *adata) {
	const int num_cols = idxij->nc;
	const double *p_points = p+num_cols*num_params_per_cam;

	static const int size_A = num_params_per_measuremnt*num_params_per_cam;
	static const int size_B = num_params_per_measuremnt*num_params_per_point;
	static const int size_AB = size_A+size_B;

	for(int j=0; j<num_cols; ++j) {
		// get j-th camera parameters
		const double *p_quat = p + j*num_params_per_cam;
		const double *p_transl = p_quat + 3;	// rotation vector part has 3 elements

		// find number of nonzero hx_ij, i=0...n-1
		int num_nonzero = sba_crsm_col_elmidxs(idxij, j, rcidxs, rcsubs);
		for(int i=0; i<num_nonzero; ++i) {
			const double *p_point = p_points + rcsubs[i]*num_params_per_point;
			double *p_Aij = jac + idxij->val[rcidxs[i]]*size_AB;
			double *p_Bij = p_Aij + size_A;

			ideal_pinhole_model_quatvec_jac<double>(p_point,
				p_quat,
				p_transl,
				(double (*)[6])p_Aij,
				(double (*)[3])p_Bij);
		}
	}
}

void sba_pinhole_model_jac(double *p, struct sba_crsm *idxij, int *rcidxs, int *rcsubs, double *jac, void *adata) {
	const sba_model_data_t<double> *sba_data = static_cast< sba_model_data_t<double>* >(adata);
	if(!sba_data) return;

	const cv::Mat &camera_matrix = sba_data->camera_matrix;

	const int num_cols = idxij->nc;
	double *p_pose = p;
	double *p_points = p+num_cols*num_params_per_cam;

	static const int size_A = num_params_per_measuremnt*num_params_per_cam;
	static const int size_B = num_params_per_measuremnt*num_params_per_point;
	static const int size_AB = size_A+size_B;
	
	for(int j=0; j<num_cols; ++j) {
		// get j-th camera parameters
		double *p_quat = p_pose + j*num_params_per_cam;
		double *p_transl = p_quat + 3;	// rotation vector part has 3 elements

		// find number of nonzero hx_ij, i=0...n-1 
		int num_nonzero = sba_crsm_col_elmidxs(idxij, j, rcidxs, rcsubs); 
		for(int i=0; i<num_nonzero; ++i) {
			double *p_point = p_points + rcsubs[i]*num_params_per_point;
			double *p_Aij = jac + idxij->val[rcidxs[i]]*size_AB;
			double *p_Bij = p_Aij + size_A;

			pinhole_model_quatvec_jac<double>(p_point,
				p_quat,
				p_transl,
				camera_matrix,
				(double (*)[6])p_Aij,
				(double (*)[3])p_Bij);
		}
	}
}

int sba_motstr_levmar_w(
		const int n,
		const int ncon,
		const int m,
		const int mcon,
		char *vmask,
		double *p,
		const int cnp,
		const int pnp,
		double *x,
		double *covx,
		const int mnp,
		void (*func)(double *p, struct sba_crsm *idxij, int *rcidxs, int *rcsubs, double *hx, void *adata),
		void (*fjac)(double *p, struct sba_crsm *idxij, int *rcidxs, int *rcsubs, double *jac, void *adata),
		void (*fweight)(double *r, const unsigned int n),
		void *adata,
		const int itmax,
		const int verbose,
		const double opts[SBA_OPTSSZ],
		double info[SBA_INFOSZ]) {

	register int i, j, ii, jj, k, l;
	int nvis, nnz, retval;

	/* The following are work arrays that are dynamically allocated by sba_motstr_levmar_x() */
	double *jac;  /* work array for storing the jacobian, max. size n*m*(mnp*cnp + mnp*pnp) */
	double *U;    /* work array for storing the U_j in the order U_1, ..., U_m, size m*cnp*cnp */
	double *V;    /* work array for storing the *strictly upper triangles* of V_i in the order V_1, ..., V_n, size n*pnp*pnp.
	               * V also stores the lower triangles of (V*_i)^-1 in the order (V*_1)^-1, ..., (V*_n)^-1.
	               * Note that diagonal elements of V_1 are saved in diagUV
	               */
	double *e;    /* work array for storing the e_ij in the order e_11, ..., e_1m, ..., e_n1, ..., e_nm, max. size n*m*mnp */
	double *eab;  /* work array for storing the ea_j & eb_i in the order ea_1, .. ea_m eb_1, .. eb_n size m*cnp + n*pnp */
	double *E;   /* work array for storing the e_j in the order e_1, .. e_m, size m*cnp */

	/* Notice that the blocks W_ij, Y_ij are zero iff A_ij (equivalently B_ij) is zero. This means
	* that the matrix consisting of blocks W_ij is itself sparse, similarly to the
	* block matrix made up of the A_ij and B_ij (i.e. jac)
	*/
	double *W;    /* work array for storing the W_ij in the order W_11, ..., W_1m, ..., W_n1, ..., W_nm,
	                 max. size n*m*cnp*pnp */
	double *Yj;   /* work array for storing the Y_ij for a *fixed* j in the order Y_1j, Y_nj,
	                 max. size n*cnp*pnp */
	double *YWt;  /* work array for storing \sum_i Y_ij W_ik^T, size cnp*cnp */
	double *S;    /* work array for storing the block array S_jk, size m*m*cnp*cnp */
	double *dp;   /* work array for storing the parameter vector updates da_1, ..., da_m, db_1, ..., db_n, size m*cnp + n*pnp */
	double *Wtda; /* work array for storing \sum_j W_ij^T da_j, size pnp */
	double *wght= /* work array for storing the weights computed from the covariance inverses, max. size n*m*mnp*mnp */
		NULL;

	/* Of the above arrays, jac, e, W, Yj, wght are sparse and
	* U, V, eab, E, S, dp are dense. Sparse arrays (except Yj) are indexed
	* through idxij (see below), that is with the same mechanism as the input
	* measurements vector x
	*/
	double *ea, *eb, *dpa, *dpb; /* pointers into p, jac, eab and dp respectively */

	/* submatrices sizes */
	int Asz, Bsz, ABsz, Usz, Vsz,
		Wsz, Ysz, esz, easz, ebsz,
		YWtsz, Wtdasz, Sblsz, covsz;

	int Sdim; /* S matrix actual dimension */

	register double *ptr1, *ptr2, *ptr3, *ptr4, sum;
	struct sba_crsm idxij; /* sparse matrix containing the location of x_ij in x. This is also
	                        * the location of A_ij, B_ij in jac, etc.
	                        * This matrix can be thought as a map from a sparse set of pairs (i, j) to a continuous
	                        * index k and it is used to efficiently lookup the memory locations where the non-zero
	                        * blocks of a sparse matrix/vector are stored
	                        */
	int maxCvis, /* max. of projections of a single point  across cameras, <=m */
		maxPvis, /* max. of projections in a single camera across points,  <=n */
		maxCPvis, /* max. of the above */
		*rcidxs,  /* work array for the indexes corresponding to the nonzero elements of a single row or
		             column in a sparse matrix, size max(n, m) */
		*rcsubs;  /* work array for the subscripts of nonzero elements in a single row or column of a
		             sparse matrix, size max(n, m) */

	/* The following variables are needed by the LM algorithm */
	register int itno;  /* iteration counter */
	int issolved;
	/* temporary work arrays that are dynamically allocated */
	double *hx,         /* \hat{x}_i, max. size m*n*mnp */
		*diagUV,     /* diagonals of U_j, V_i, size m*cnp + n*pnp */
		*pdp;        /* p + dp, size m*cnp + n*pnp */

	double *diagU, *diagV; /* pointers into diagUV */

	register double mu,  /* damping constant */
		tmp; /* mainly used in matrix & vector multiplications */
	double p_eL2, eab_inf, pdp_eL2; /* ||e(p)||_2, ||J^T e||_inf, ||e(p+dp)||_2 */
	double p_L2, dp_L2=DBL_MAX, dF, dL;
	double tau=FABS(opts[0]), eps1=FABS(opts[1]), eps2=FABS(opts[2]), eps2_sq=opts[2]*opts[2],
		eps3_sq=opts[3]*opts[3], eps4_sq=opts[4]*opts[4];
	double init_p_eL2;
	int nu=2, nu2, stop=0, nfev, njev=0, nlss=0;
	int nobs, nvars;
	const int mmcon=m-mcon;
	PLS linsolver=NULL;
	int (*matinv)(double *A, int m)=NULL;

	struct fdj_data_x_ fdj_data;
	void *jac_adata;

	/* Initialization */
	mu=eab_inf=0.0; /* -Wall */

	/* block sizes */
	Asz=mnp * cnp; Bsz=mnp * pnp; ABsz=Asz + Bsz;
	Usz=cnp * cnp; Vsz=pnp * pnp;
	Wsz=cnp * pnp; Ysz=cnp * pnp;
	esz=mnp;
	easz=cnp; ebsz=pnp;
	YWtsz=cnp * cnp;
	Wtdasz=pnp;
	Sblsz=cnp * cnp;
	Sdim=mmcon * cnp;
	covsz=mnp * mnp;

	/* count total number of visible image points */
	for(i=nvis=0, jj=n*m; i<jj; ++i)
		nvis+=(vmask[i]!=0);

	nobs=nvis*mnp;
	nvars=m*cnp + n*pnp;
	if(nobs<nvars){
	fprintf(stderr, "SBA: sba_motstr_levmar_x() cannot solve a problem with fewer measurements [%d] than unknowns [%d]\n", nobs, nvars);
	return SBA_ERROR;
	}

	/* allocate & fill up the idxij structure */
	sba_crsm_alloc(&idxij, n, m, nvis);
	for(i=k=0; i<n; ++i){
		idxij.rowptr[i]=k;
		ii=i*m;
		for(j=0; j<m; ++j)
			if(vmask[ii+j]){
				idxij.val[k]=k;
				idxij.colidx[k++]=j;
			}
	}
	idxij.rowptr[n]=nvis;

	/* find the maximum number (for all cameras) of visible image projections coming from a single 3D point */
	for(i=maxCvis=0; i<n; ++i)
		if((k=idxij.rowptr[i+1]-idxij.rowptr[i])>maxCvis) maxCvis=k;

	/* find the maximum number (for all points) of visible image projections in any single camera */
	for(j=maxPvis=0; j<m; ++j){
		for(i=ii=0; i<n; ++i)
			if(vmask[i*m+j]) ++ii;
			if(ii>maxPvis) maxPvis=ii;
	}
	maxCPvis=(maxCvis>=maxPvis)? maxCvis : maxPvis;

#if 0
	/* determine the density of blocks in matrix S */
	for(j=mcon, ii=0; j<m; ++j){
		++ii; /* block Sjj is surely nonzero */
		for(k=j+1; k<m; ++k)
			if(sba_crsm_common_row(&idxij, j, k)) ii+=2; /* blocks Sjk & Skj are nonzero */
	}
	printf("\nS block density: %.5g\n", ((double)ii)/(mmcon*mmcon)); fflush(stdout);
#endif

	/* allocate work arrays */
	/* W is big enough to hold both jac & W. Note also the extra Wsz, see the initialization of jac below for explanation */
	W=(double *)emalloc((nvis*((Wsz>=ABsz)? Wsz : ABsz) + Wsz)*sizeof(double));
	U=(double *)emalloc(m*Usz*sizeof(double));
	V=(double *)emalloc(n*Vsz*sizeof(double));
	e=(double *)emalloc(nobs*sizeof(double));
	eab=(double *)emalloc(nvars*sizeof(double));
	E=(double *)emalloc(m*cnp*sizeof(double));
	Yj=(double *)emalloc(maxPvis*Ysz*sizeof(double));
	YWt=(double *)emalloc(YWtsz*sizeof(double));
	S=(double *)emalloc(m*m*Sblsz*sizeof(double));
	dp=(double *)emalloc(nvars*sizeof(double));
	Wtda=(double *)emalloc(pnp*sizeof(double));
	rcidxs=(int *)emalloc(maxCPvis*sizeof(int));
	rcsubs=(int *)emalloc(maxCPvis*sizeof(int));
#ifndef SBA_DESTROY_COVS
	if(covx!=NULL) wght=(double *)emalloc(nvis*covsz*sizeof(double));
#else
	if(covx!=NULL) wght=covx;
#endif /* SBA_DESTROY_COVS */


	hx=(double *)emalloc(nobs*sizeof(double));
	diagUV=(double *)emalloc(nvars*sizeof(double));
	pdp=(double *)emalloc(nvars*sizeof(double));

	/* to save resources, W and jac share the same memory: First, the jacobian
	 * is computed in some working memory that is then overwritten during the
	 * computation of W. To account for the case of W being larger than jac,
	 * extra memory is reserved "before" jac.
	 * Care must be taken, however, to ensure that storing a certain W_ij
	 * does not overwrite the A_ij, B_ij used to compute it. To achieve
	 * this is, note that if p1 and p2 respectively point to the first elements
	 * of a certain W_ij and A_ij, B_ij pair, we should have p2-p1>=Wsz.
	 * There are two cases:
	 * a) Wsz>=ABsz: Then p1=W+k*Wsz and p2=jac+k*ABsz=W+Wsz+nvis*(Wsz-ABsz)+k*ABsz
	 *    for some k (0<=k<nvis), thus p2-p1=(nvis-k)*(Wsz-ABsz)+Wsz.
	 *    The right side of the last equation is obviously > Wsz for all 0<=k<nvis
	 *
	 * b) Wsz<ABsz: Then p1=W+k*Wsz and p2=jac+k*ABsz=W+Wsz+k*ABsz and
	 *    p2-p1=Wsz+k*(ABsz-Wsz), which is again > Wsz for all 0<=k<nvis
	 *
	 * In conclusion, if jac is initialized as below, the memory allocated to all
	 * W_ij is guaranteed not to overlap with that allocated to their corresponding
	 * A_ij, B_ij pairs
	 */
	jac=W + Wsz + ((Wsz>ABsz)? nvis*(Wsz-ABsz) : 0);

	/* set up auxiliary pointers */
	ea=eab; eb=eab+m*cnp;
	dpa=dp; dpb=dp+m*cnp;

	diagU=diagUV; diagV=diagUV + m*cnp;

	/* if no jacobian function is supplied, prepare to compute jacobian with finite difference */
	if(!fjac){
		fdj_data.func=func;
		fdj_data.cnp=cnp;
		fdj_data.pnp=pnp;
		fdj_data.mnp=mnp;
		fdj_data.hx=hx;
		fdj_data.hxx=(double *)emalloc(nobs*sizeof(double));
		fdj_data.func_rcidxs=(int *)emalloc(2*maxCPvis*sizeof(int));
		fdj_data.func_rcsubs=fdj_data.func_rcidxs+maxCPvis;
		fdj_data.adata=adata;

		fjac=sba_fdjac_x;
		jac_adata=(void *)&fdj_data;
	}
	else{
		fdj_data.hxx=NULL;
		jac_adata=adata;
	}

	if(itmax==0){ /* verify jacobian */
		sba_motstr_chkjac_x(func, fjac, p, &idxij, rcidxs, rcsubs, ncon, mcon, cnp, pnp, mnp, adata, jac_adata);
		retval=0;
		goto freemem_and_return;
	}

	/* covariances Sigma_x_ij are accommodated by computing the Cholesky decompositions of their
	* inverses and using the resulting matrices w_x_ij to weigh A_ij, B_ij, and e_ij as w_x_ij A_ij,
	* w_x_ij*B_ij and w_x_ij*e_ij. In this way, auxiliary variables as U_j=\sum_i A_ij^T A_ij
	* actually become \sum_i (w_x_ij A_ij)^T w_x_ij A_ij= \sum_i A_ij^T w_x_ij^T w_x_ij A_ij =
	* A_ij^T Sigma_x_ij^-1 A_ij
	*
	* ea_j, V_i, eb_i, W_ij are weighted in a similar manner
	*/
	if(covx!=NULL){
		for(i=0; i<n; ++i){
			nnz=sba_crsm_row_elmidxs(&idxij, i, rcidxs, rcsubs); /* find nonzero x_ij, j=0...m-1 */
			for(j=0; j<nnz; ++j){
				/* set ptr1, ptr2 to point to cov_x_ij, w_x_ij resp. */
				ptr1=covx + (k=idxij.val[rcidxs[j]]*covsz);
				ptr2=wght + k;
				if(!sba_mat_cholinv(ptr1, ptr2, mnp)){ /* compute w_x_ij s.t. w_x_ij^T w_x_ij = cov_x_ij^-1 */
					fprintf(stderr, "SBA: invalid covariance matrix for x_ij (i=%d, j=%d) in sba_motstr_levmar_x()\n", i, rcsubs[j]);
					retval=SBA_ERROR;
					goto freemem_and_return;
				}
			}
		}
		sba_mat_cholinv(NULL, NULL, 0); /* cleanup */
	}

	/* compute the error vectors e_ij in hx */
	(*func)(p, &idxij, rcidxs, rcsubs, hx, adata); nfev=1;
	/* ### compute e=x - f(p) [e=w*(x - f(p)] and its L2 norm */
	if(covx==NULL)
		p_eL2=nrmL2xmy(e, x, hx, nobs); /* e=x-hx, p_eL2=||e|| */
	else
		p_eL2=nrmCxmy(e, x, hx, wght, mnp, nvis); /* e=wght*(x-hx), p_eL2=||e||=||x-hx||_Sigma^-1 */
	if(verbose) printf("initial motstr-SBA error %g [%g]\n", p_eL2, p_eL2/nvis);
		init_p_eL2=p_eL2;
		if(!SBA_FINITE(p_eL2)) stop=7;

	for(itno=0; itno<itmax && !stop; ++itno){
		/* Note that p, e and ||e||_2 have been updated at the previous iteration */

		/* compute derivative submatrices A_ij, B_ij */
		(*fjac)(p, &idxij, rcidxs, rcsubs, jac, jac_adata); ++njev;

		if(covx!=NULL){
			/* compute w_x_ij A_ij and w_x_ij B_ij.
			 * Since w_x_ij is upper triangular, the products can be safely saved
			 * directly in A_ij, B_ij, without the need for intermediate storage
			 */
			for(i=0; i<nvis; ++i){
				/* set ptr1, ptr2, ptr3 to point to w_x_ij, A_ij, B_ij, resp. */
				ptr1=wght + i*covsz;
				ptr2=jac  + i*ABsz;
				ptr3=ptr2 + Asz; // ptr3=jac  + i*ABsz + Asz;

				/* w_x_ij is mnp x mnp, A_ij is mnp x cnp, B_ij is mnp x pnp
				* NOTE: Jamming the outter (i.e., ii) loops did not run faster!
				*/
				/* A_ij */
				for(ii=0; ii<mnp; ++ii)
					for(jj=0; jj<cnp; ++jj){
						for(k=ii, sum=0.0; k<mnp; ++k) // k>=ii since w_x_ij is upper triangular
							sum+=ptr1[ii*mnp+k]*ptr2[k*cnp+jj];
						ptr2[ii*cnp+jj]=sum;
					}

				/* B_ij */
				for(ii=0; ii<mnp; ++ii)
					for(jj=0; jj<pnp; ++jj){
						for(k=ii, sum=0.0; k<mnp; ++k) // k>=ii since w_x_ij is upper triangular
							sum+=ptr1[ii*mnp+k]*ptr3[k*pnp+jj];
						ptr3[ii*pnp+jj]=sum;
					}
			}
		}

		/* compute U_j = \sum_i A_ij^T A_ij */ // \Sigma here!
		/* U_j is symmetric, therefore its computation can be sped up by
		* computing only the upper part and then reusing it for the lower one.
		* Recall that A_ij is mnp x cnp
		*/
		/* Also compute ea_j = \sum_i A_ij^T e_ij */ // \Sigma here!
		/* Recall that e_ij is mnp x 1
		*/
		_dblzero(U, m*Usz); /* clear all U_j */
		_dblzero(ea, m*easz); /* clear all ea_j */
		for(j=mcon; j<m; ++j){
			ptr1=U + j*Usz; // set ptr1 to point to U_j
			ptr2=ea + j*easz; // set ptr2 to point to ea_j

			nnz=sba_crsm_col_elmidxs(&idxij, j, rcidxs, rcsubs); /* find nonzero A_ij, i=0...n-1 */
			for(i=0; i<nnz; ++i){
				/* set ptr3 to point to A_ij, actual row number in rcsubs[i] */
				ptr3=jac + idxij.val[rcidxs[i]]*ABsz;

				/* compute the UPPER TRIANGULAR PART of A_ij^T A_ij and add it to U_j */
				for(ii=0; ii<cnp; ++ii){
					for(jj=ii; jj<cnp; ++jj){
						for(k=0, sum=0.0; k<mnp; ++k)
							sum+=ptr3[k*cnp+ii]*ptr3[k*cnp+jj];
						ptr1[ii*cnp+jj]+=sum;
					}

					/* copy the LOWER TRIANGULAR PART of U_j from the upper one */
					for(jj=0; jj<ii; ++jj)
						ptr1[ii*cnp+jj]=ptr1[jj*cnp+ii];
				}

				ptr4=e + idxij.val[rcidxs[i]]*esz; /* set ptr4 to point to e_ij */
				/* compute A_ij^T e_ij and add it to ea_j */
				for(ii=0; ii<cnp; ++ii){
					for(jj=0, sum=0.0; jj<mnp; ++jj)
						sum+=ptr3[jj*cnp+ii]*ptr4[jj];
					ptr2[ii]+=sum;
				}
			}
		}

		/* compute V_i = \sum_j B_ij^T B_ij */ // \Sigma here!
		/* V_i is symmetric, therefore its computation can be sped up by
		* computing only the upper part and then reusing it for the lower one.
		* Recall that B_ij is mnp x pnp
		*/
		/* Also compute eb_i = \sum_j B_ij^T e_ij */ // \Sigma here!
		/* Recall that e_ij is mnp x 1
		*/
		_dblzero(V, n*Vsz); /* clear all V_i */
		_dblzero(eb, n*ebsz); /* clear all eb_i */
		for(i=ncon; i<n; ++i){
			ptr1=V + i*Vsz; // set ptr1 to point to V_i
			ptr2=eb + i*ebsz; // set ptr2 to point to eb_i

			nnz=sba_crsm_row_elmidxs(&idxij, i, rcidxs, rcsubs); /* find nonzero B_ij, j=0...m-1 */
			for(j=0; j<nnz; ++j){
				/* set ptr3 to point to B_ij, actual column number in rcsubs[j] */
				ptr3=jac + idxij.val[rcidxs[j]]*ABsz + Asz;

				/* compute the UPPER TRIANGULAR PART of B_ij^T B_ij and add it to V_i */
				for(ii=0; ii<pnp; ++ii){
					for(jj=ii; jj<pnp; ++jj){
						for(k=0, sum=0.0; k<mnp; ++k)
							sum+=ptr3[k*pnp+ii]*ptr3[k*pnp+jj];
						ptr1[ii*pnp+jj]+=sum;
					}
				}

				ptr4=e + idxij.val[rcidxs[j]]*esz; /* set ptr4 to point to e_ij */
				/* compute B_ij^T e_ij and add it to eb_i */
				for(ii=0; ii<pnp; ++ii){
					for(jj=0, sum=0.0; jj<mnp; ++jj)
						sum+=ptr3[jj*pnp+ii]*ptr4[jj];
					ptr2[ii]+=sum;
				}
			}
		}

		/* compute W_ij =  A_ij^T B_ij */ // \Sigma here!
		/* Recall that A_ij is mnp x cnp and B_ij is mnp x pnp
		*/
		for(i=ncon; i<n; ++i){
			nnz=sba_crsm_row_elmidxs(&idxij, i, rcidxs, rcsubs); /* find nonzero W_ij, j=0...m-1 */
			for(j=0; j<nnz; ++j){
				/* set ptr1 to point to W_ij, actual column number in rcsubs[j] */
				ptr1=W + idxij.val[rcidxs[j]]*Wsz;

				if(rcsubs[j]<mcon){ /* A_ij is zero */
					//_dblzero(ptr1, Wsz); /* clear W_ij */
					continue;
				}

				/* set ptr2 & ptr3 to point to A_ij & B_ij resp. */
				ptr2=jac  + idxij.val[rcidxs[j]]*ABsz;
				ptr3=ptr2 + Asz;
				/* compute A_ij^T B_ij and store it in W_ij
				* Recall that storage for A_ij, B_ij does not overlap with that for W_ij,
				* see the comments related to the initialization of jac above
				*/
				/* assert(ptr2-ptr1>=Wsz); */
				for(ii=0; ii<cnp; ++ii)
					for(jj=0; jj<pnp; ++jj){
						for(k=0, sum=0.0; k<mnp; ++k)
							sum+=ptr2[k*cnp+ii]*ptr3[k*pnp+jj];
						ptr1[ii*pnp+jj]=sum;
					}
			}
		}

		/* Compute ||J^T e||_inf and ||p||^2 */
		for(i=0, p_L2=eab_inf=0.0; i<nvars; ++i){
			if(eab_inf < (tmp=FABS(eab[i]))) eab_inf=tmp;
			p_L2+=p[i]*p[i];
		}
		//p_L2=sqrt(p_L2);

		/* save diagonal entries so that augmentation can be later canceled.
		* Diagonal entries are in U_j and V_i
		*/
		for(j=mcon; j<m; ++j){
			ptr1=U + j*Usz; // set ptr1 to point to U_j
			ptr2=diagU + j*cnp; // set ptr2 to point to diagU_j
			for(i=0; i<cnp; ++i)
				ptr2[i]=ptr1[i*cnp+i];
		}
		for(i=ncon; i<n; ++i){
			ptr1=V + i*Vsz; // set ptr1 to point to V_i
			ptr2=diagV + i*pnp; // set ptr2 to point to diagV_i
			for(j=0; j<pnp; ++j)
				ptr2[j]=ptr1[j*pnp+j];
		}

/*
		if(!(itno%100)){
			printf("Current estimate: ");
			for(i=0; i<nvars; ++i)
			printf("%.9g ", p[i]);
			printf("-- errors %.9g %0.9g\n", eab_inf, p_eL2);
		}
*/

		/* check for convergence */
		if((eab_inf <= eps1)){
			dp_L2=0.0; /* no increment for p in this case */
			stop=1;
			break;
		}

		/* compute initial damping factor */
		if(itno==0){
			/* find max diagonal element */
			for(i=mcon*cnp, tmp=DBL_MIN; i<m*cnp; ++i)
				if(diagUV[i]>tmp) tmp=diagUV[i];
			for(i=m*cnp + ncon*pnp; i<nvars; ++i) /* tmp is not re-initialized! */
				if(diagUV[i]>tmp) tmp=diagUV[i];
			mu=tau*tmp;
		}

		/* determine increment using adaptive damping */
		while(1){
			/* augment U, V */
			for(j=mcon; j<m; ++j){
				ptr1=U + j*Usz; // set ptr1 to point to U_j
				for(i=0; i<cnp; ++i)
					ptr1[i*cnp+i]+=mu;
			}
			for(i=ncon; i<n; ++i){
				ptr1=V + i*Vsz; // set ptr1 to point to V_i
				for(j=0; j<pnp; ++j)
					ptr1[j*pnp+j]+=mu;

				/* compute V*_i^-1.
				* Recall that only the upper triangle of the symmetric pnp x pnp matrix V*_i
				* is stored in ptr1; its (also symmetric) inverse is saved in the lower triangle of ptr1
				*/
				/* inverting V*_i with LDLT seems to result in faster overall execution compared to when using LU or Cholesky */
				//j=sba_symat_invert_LU(ptr1, pnp); matinv=sba_symat_invert_LU;
				//j=sba_symat_invert_Chol(ptr1, pnp); matinv=sba_symat_invert_Chol;
				j=sba_symat_invert_BK(ptr1, pnp); matinv=sba_symat_invert_BK;
				if(!j){
					fprintf(stderr, "SBA: singular matrix V*_i (i=%d) in sba_motstr_levmar_x(), increasing damping\n", i);
					goto moredamping; // increasing damping will eventually make V*_i diagonally dominant, thus nonsingular
					//retval=SBA_ERROR;
					//goto freemem_and_return;
				}
			}

			_dblzero(E, m*easz); /* clear all e_j */
			/* compute the mmcon x mmcon block matrix S and e_j */

			/* Note that S is symmetric, therefore its computation can be
			* sped up by computing only the upper part and then reusing
			* it for the lower one.
			*/
			for(j=mcon; j<m; ++j){
				int mmconxUsz=mmcon*Usz;

				nnz=sba_crsm_col_elmidxs(&idxij, j, rcidxs, rcsubs); /* find nonzero Y_ij, i=0...n-1 */

				/* get rid of all Y_ij with i<ncon that are treated as zeros.
				* In this way, all rcsubs[i] below are guaranteed to be >= ncon
				*/
				if(ncon){
					for(i=ii=0; i<nnz; ++i){
						if(rcsubs[i]>=ncon){
							rcidxs[ii]=rcidxs[i];
							rcsubs[ii++]=rcsubs[i];
						}
					}
					nnz=ii;
				}

				/* compute all Y_ij = W_ij (V*_i)^-1 for a *fixed* j.
				* To save memory, the block matrix consisting of the Y_ij
				* is not stored. Instead, only a block column of this matrix
				* is computed & used at each time: For each j, all nonzero
				* Y_ij are computed in Yj and then used in the calculations
				* involving S_jk and e_j.
				* Recall that W_ij is cnp x pnp and (V*_i) is pnp x pnp
				*/
				for(i=0; i<nnz; ++i){
					/* set ptr3 to point to (V*_i)^-1, actual row number in rcsubs[i] */
					ptr3=V + rcsubs[i]*Vsz;

					/* set ptr1 to point to Y_ij, actual row number in rcsubs[i] */
					ptr1=Yj + i*Ysz;
					/* set ptr2 to point to W_ij resp. */
					ptr2=W + idxij.val[rcidxs[i]]*Wsz;
					/* compute W_ij (V*_i)^-1 and store it in Y_ij.
					* Recall that only the lower triangle of (V*_i)^-1 is stored
					*/
					for(ii=0; ii<cnp; ++ii){
						ptr4=ptr2+ii*pnp;
						for(jj=0; jj<pnp; ++jj){
							for(k=0, sum=0.0; k<=jj; ++k)
								sum+=ptr4[k]*ptr3[jj*pnp+k]; //ptr2[ii*pnp+k]*ptr3[jj*pnp+k];
							for( ; k<pnp; ++k)
								sum+=ptr4[k]*ptr3[k*pnp+jj]; //ptr2[ii*pnp+k]*ptr3[k*pnp+jj];
							ptr1[ii*pnp+jj]=sum;
						}
					}
				}

				/* compute the UPPER TRIANGULAR PART of S */
				for(k=j; k<m; ++k){ // j>=mcon
					/* compute \sum_i Y_ij W_ik^T in YWt. Note that for an off-diagonal block defined by j, k
					* YWt (and thus S_jk) is nonzero only if there exists a point that is visible in both the
					* j-th and k-th images
					*/

					/* Recall that Y_ij is cnp x pnp and W_ik is cnp x pnp */
					_dblzero(YWt, YWtsz); /* clear YWt */

					for(i=0; i<nnz; ++i){
						register double *pYWt;

						/* find the min and max column indices of the elements in row i (actually rcsubs[i])
						* and make sure that k falls within them. This test handles W_ik's which are
						* certain to be zero without bothering to call sba_crsm_elmidx()
						*/
						ii=idxij.colidx[idxij.rowptr[rcsubs[i]]];
						jj=idxij.colidx[idxij.rowptr[rcsubs[i]+1]-1];
						if(k<ii || k>jj) continue; /* W_ik == 0 */

						/* set ptr2 to point to W_ik */
						l=sba_crsm_elmidxp(&idxij, rcsubs[i], k, j, rcidxs[i]);
						//l=sba_crsm_elmidx(&idxij, rcsubs[i], k);
						if(l==-1) continue; /* W_ik == 0 */

						ptr2=W + idxij.val[l]*Wsz;
						/* set ptr1 to point to Y_ij, actual row number in rcsubs[i] */
						ptr1=Yj + i*Ysz;
						for(ii=0; ii<cnp; ++ii){
							ptr3=ptr1+ii*pnp;
							pYWt=YWt+ii*cnp;

							for(jj=0; jj<cnp; ++jj){
								ptr4=ptr2+jj*pnp;
								for(l=0, sum=0.0; l<pnp; ++l)
									sum+=ptr3[l]*ptr4[l]; //ptr1[ii*pnp+l]*ptr2[jj*pnp+l];
								pYWt[jj]+=sum; //YWt[ii*cnp+jj]+=sum;
							}
						}
					}

					/* since the linear system involving S is solved with lapack,
					* it is preferable to store S in column major (i.e. fortran)
					* order, so as to avoid unecessary transposing/copying.
					*/
#if MAT_STORAGE==COLUMN_MAJOR
					ptr2=S + (k-mcon)*mmconxUsz + (j-mcon)*cnp; // set ptr2 to point to the beginning of block j,k in S
#else
					ptr2=S + (j-mcon)*mmconxUsz + (k-mcon)*cnp; // set ptr2 to point to the beginning of block j,k in S
#endif

					if(j!=k){ /* Kronecker */
						for(ii=0; ii<cnp; ++ii, ptr2+=Sdim)
							for(jj=0; jj<cnp; ++jj)
								ptr2[jj]=
#if MAT_STORAGE==COLUMN_MAJOR
									-YWt[jj*cnp+ii];
#else
									-YWt[ii*cnp+jj];
#endif
					}
					else{
						ptr1=U + j*Usz; // set ptr1 to point to U_j

						for(ii=0; ii<cnp; ++ii, ptr2+=Sdim)
							for(jj=0; jj<cnp; ++jj)
						ptr2[jj]=
#if MAT_STORAGE==COLUMN_MAJOR
								ptr1[jj*cnp+ii] - YWt[jj*cnp+ii];
#else
								ptr1[ii*cnp+jj] - YWt[ii*cnp+jj];
#endif
					}
				}

				/* copy the LOWER TRIANGULAR PART of S from the upper one */
				for(k=mcon; k<j; ++k){
#if MAT_STORAGE==COLUMN_MAJOR
					ptr1=S + (k-mcon)*mmconxUsz + (j-mcon)*cnp; // set ptr1 to point to the beginning of block j,k in S
					ptr2=S + (j-mcon)*mmconxUsz + (k-mcon)*cnp; // set ptr2 to point to the beginning of block k,j in S
#else
					ptr1=S + (j-mcon)*mmconxUsz + (k-mcon)*cnp; // set ptr1 to point to the beginning of block j,k in S
					ptr2=S + (k-mcon)*mmconxUsz + (j-mcon)*cnp; // set ptr2 to point to the beginning of block k,j in S
#endif
					for(ii=0; ii<cnp; ++ii, ptr1+=Sdim)
						for(jj=0, ptr3=ptr2+ii; jj<cnp; ++jj, ptr3+=Sdim)
							ptr1[jj]=*ptr3;
				}

				/* compute e_j=ea_j - \sum_i Y_ij eb_i */
				/* Recall that Y_ij is cnp x pnp and eb_i is pnp x 1 */
				ptr1=E + j*easz; // set ptr1 to point to e_j

				for(i=0; i<nnz; ++i){
					/* set ptr2 to point to Y_ij, actual row number in rcsubs[i] */
					ptr2=Yj + i*Ysz;

					/* set ptr3 to point to eb_i */
					ptr3=eb + rcsubs[i]*ebsz;
					for(ii=0; ii<cnp; ++ii){
						ptr4=ptr2+ii*pnp;
						for(jj=0, sum=0.0; jj<pnp; ++jj)
							sum+=ptr4[jj]*ptr3[jj]; //ptr2[ii*pnp+jj]*ptr3[jj];
						ptr1[ii]+=sum;
					}
				}

				ptr2=ea + j*easz; // set ptr2 to point to ea_j
				for(i=0; i<easz; ++i)
					ptr1[i]=ptr2[i] - ptr1[i];
			}

#if 0
			if(verbose>1){ /* count the nonzeros in S */
				for(i=ii=0; i<Sdim*Sdim; ++i)
					if(S[i]!=0.0) ++ii;
				printf("\nS density: %.5g\n", ((double)ii)/(Sdim*Sdim)); fflush(stdout);
			}
#endif

			/* solve the linear system S dpa = E to compute the da_j.
			*
			* Note that if MAT_STORAGE==1 S is modified in the following call;
			* this is OK since S is recomputed for each iteration
			*/
			//issolved=sba_Axb_LU(S, E+mcon*cnp, dpa+mcon*cnp, Sdim, MAT_STORAGE); linsolver=sba_Axb_LU;
			issolved=sba_Axb_Chol(S, E+mcon*cnp, dpa+mcon*cnp, Sdim, MAT_STORAGE); linsolver=sba_Axb_Chol;
			//issolved=sba_Axb_BK(S, E+mcon*cnp, dpa+mcon*cnp, Sdim, MAT_STORAGE); linsolver=sba_Axb_BK;
			//issolved=sba_Axb_QRnoQ(S, E+mcon*cnp, dpa+mcon*cnp, Sdim, MAT_STORAGE); linsolver=sba_Axb_QRnoQ;
			//issolved=sba_Axb_QR(S, E+mcon*cnp, dpa+mcon*cnp, Sdim, MAT_STORAGE); linsolver=sba_Axb_QR;
			//issolved=sba_Axb_SVD(S, E+mcon*cnp, dpa+mcon*cnp, Sdim, MAT_STORAGE); linsolver=sba_Axb_SVD;
			//issolved=sba_Axb_CG(S, E+mcon*cnp, dpa+mcon*cnp, Sdim, (3*Sdim)/2, 1E-10, SBA_CG_JACOBI, MAT_STORAGE); linsolver=(PLS)sba_Axb_CG;

			++nlss;

			_dblzero(dpa, mcon*cnp); /* no change for the first mcon camera params */

			if(issolved){

				/* compute the db_i */
				for(i=ncon; i<n; ++i){
					ptr1=dpb + i*ebsz; // set ptr1 to point to db_i

					/* compute \sum_j W_ij^T da_j */
					/* Recall that W_ij is cnp x pnp and da_j is cnp x 1 */
					_dblzero(Wtda, Wtdasz); /* clear Wtda */
					nnz=sba_crsm_row_elmidxs(&idxij, i, rcidxs, rcsubs); /* find nonzero W_ij, j=0...m-1 */
					for(j=0; j<nnz; ++j){
						/* set ptr2 to point to W_ij, actual column number in rcsubs[j] */
						if(rcsubs[j]<mcon) continue; /* W_ij is zero */

						ptr2=W + idxij.val[rcidxs[j]]*Wsz;

						/* set ptr3 to point to da_j */
						ptr3=dpa + rcsubs[j]*cnp;

						for(ii=0; ii<pnp; ++ii){
							ptr4=ptr2+ii;
							for(jj=0, sum=0.0; jj<cnp; ++jj)
								sum+=ptr4[jj*pnp]*ptr3[jj]; //ptr2[jj*pnp+ii]*ptr3[jj];
							Wtda[ii]+=sum;
						}
					}

					/* compute eb_i - \sum_j W_ij^T da_j = eb_i - Wtda in Wtda */
					ptr2=eb + i*ebsz; // set ptr2 to point to eb_i
					for(ii=0; ii<pnp; ++ii)
						Wtda[ii]=ptr2[ii] - Wtda[ii];

					/* compute the product (V*_i)^-1 Wtda = (V*_i)^-1 (eb_i - \sum_j W_ij^T da_j).
					* Recall that only the lower triangle of (V*_i)^-1 is stored
					*/
					ptr2=V + i*Vsz; // set ptr2 to point to (V*_i)^-1
					for(ii=0; ii<pnp; ++ii){
						for(jj=0, sum=0.0; jj<=ii; ++jj)
							sum+=ptr2[ii*pnp+jj]*Wtda[jj];
						for( ; jj<pnp; ++jj)
							sum+=ptr2[jj*pnp+ii]*Wtda[jj];
						ptr1[ii]=sum;
					}
				}
				_dblzero(dpb, ncon*pnp); /* no change for the first ncon point params */

				/* parameter vector updates are now in dpa, dpb */

				/* compute p's new estimate and ||dp||^2 */
				for(i=0, dp_L2=0.0; i<nvars; ++i){
					pdp[i]=p[i] + (tmp=dp[i]);
					dp_L2+=tmp*tmp;
				}
				//dp_L2=sqrt(dp_L2);

				if(dp_L2<=eps2_sq*p_L2){ /* relative change in p is small, stop */
					//if(dp_L2<=eps2*(p_L2 + eps2)){ /* relative change in p is small, stop */
					stop=2;
					break;
				}

				if(dp_L2>=(p_L2+eps2)/SBA_EPSILON_SQ){ /* almost singular */
					//if(dp_L2>=(p_L2+eps2)/SBA_EPSILON){ /* almost singular */
					fprintf(stderr, "SBA: the matrix of the augmented normal equations is almost singular in sba_motstr_levmar_x(),\n"
						"     minimization should be restarted from the current solution with an increased damping term\n");
					retval=SBA_ERROR;
					goto freemem_and_return;
				}

				(*func)(pdp, &idxij, rcidxs, rcsubs, hx, adata); ++nfev; /* evaluate function at p + dp */
				if(verbose>1)
					printf("mean reprojection error %g\n", sba_mean_repr_error(n, mnp, x, hx, &idxij, rcidxs, rcsubs));
				/* ### compute ||e(pdp)||_2 */
				if(covx==NULL) {
					pdp_eL2=nrmL2xmy(hx, x, hx, nobs); /* hx=x-hx, pdp_eL2=||hx|| */
					if(fweight) fweight(hx, nobs);
				} else
					pdp_eL2=nrmCxmy(hx, x, hx, wght, mnp, nvis); /* hx=wght*(x-hx), pdp_eL2=||hx|| */
				if(!SBA_FINITE(pdp_eL2)){
					if(verbose) /* identify the offending point projection */
						sba_print_inf(hx, m, mnp, &idxij, rcidxs, rcsubs);

					stop=7;
					break;
				}

				for(i=0, dL=0.0; i<nvars; ++i)
					dL+=dp[i]*(mu*dp[i]+eab[i]);

				dF=p_eL2-pdp_eL2;

				if(verbose>1)
					printf("\ndamping term %8g, gain ratio %8g, errors %8g / %8g = %g\n", mu, dL!=0.0? dF/dL : dF/DBL_EPSILON, p_eL2/nvis, pdp_eL2/nvis, p_eL2/pdp_eL2);

				if(dL>0.0 && dF>0.0){ /* reduction in error, increment is accepted */
					tmp=(2.0*dF/dL-1.0);
					tmp=1.0-tmp*tmp*tmp;
					mu=mu*( (tmp>=SBA_ONE_THIRD)? tmp : SBA_ONE_THIRD );
					nu=2;

					/* the test below is equivalent to the relative reduction of the RMS reprojection error: sqrt(p_eL2)-sqrt(pdp_eL2)<eps4*sqrt(p_eL2) */
					if(pdp_eL2-2.0*sqrt(p_eL2*pdp_eL2)<(eps4_sq-1.0)*p_eL2) stop=4;

					for(i=0; i<nvars; ++i) /* update p's estimate */
						p[i]=pdp[i];

					for(i=0; i<nobs; ++i) /* update e and ||e||_2 */
						e[i]=hx[i];
					p_eL2=pdp_eL2;
					break;
				}
			} /* issolved */

moredamping:
			/* if this point is reached (also via an explicit goto!), either the linear system could
			* not be solved or the error did not reduce; in any case, the increment must be rejected
			*/

			mu*=nu;
			nu2=nu<<1; // 2*nu;
			if(nu2<=nu){ /* nu has wrapped around (overflown) */
				fprintf(stderr, "SBA: too many failed attempts to increase the damping factor in sba_motstr_levmar_x()! Singular Hessian matrix?\n");
				//exit(1);
				stop=6;
				break;
			}
			nu=nu2;

#if 0
      /* restore U, V diagonal entries */
      for(j=mcon; j<m; ++j){
        ptr1=U + j*Usz; // set ptr1 to point to U_j
        ptr2=diagU + j*cnp; // set ptr2 to point to diagU_j
        for(i=0; i<cnp; ++i)
          ptr1[i*cnp+i]=ptr2[i];
      }
      for(i=ncon; i<n; ++i){
        ptr1=V + i*Vsz; // set ptr1 to point to V_i
        ptr2=diagV + i*pnp; // set ptr2 to point to diagV_i
        for(j=0; j<pnp; ++j)
          ptr1[j*pnp+j]=ptr2[j];
      }
#endif
		} /* inner while loop */

		if(p_eL2<=eps3_sq) stop=5; // error is small, force termination of outer loop
	}

	if(itno>=itmax) stop=3;

	/* restore U, V diagonal entries */
	for(j=mcon; j<m; ++j){
		ptr1=U + j*Usz; // set ptr1 to point to U_j
		ptr2=diagU + j*cnp; // set ptr2 to point to diagU_j
		for(i=0; i<cnp; ++i)
			ptr1[i*cnp+i]=ptr2[i];
	}
	for(i=ncon; i<n; ++i){
		ptr1=V + i*Vsz; // set ptr1 to point to V_i
		ptr2=diagV + i*pnp; // set ptr2 to point to diagV_i
		for(j=0; j<pnp; ++j)
			ptr1[j*pnp+j]=ptr2[j];
	}

	if(info){
		info[0]=init_p_eL2;
		info[1]=p_eL2;
		info[2]=eab_inf;
		info[3]=dp_L2;
		for(j=mcon, tmp=DBL_MIN; j<m; ++j){
			ptr1=U + j*Usz; // set ptr1 to point to U_j
			for(i=0; i<cnp; ++i)
				if(tmp<ptr1[i*cnp+i]) tmp=ptr1[i*cnp+i];
		}
		for(i=ncon; i<n; ++i){
			ptr1=V + i*Vsz; // set ptr1 to point to V_i
			for(j=0; j<pnp; ++j)
			if(tmp<ptr1[j*pnp+j]) tmp=ptr1[j*pnp+j];
		}
		info[4]=mu/tmp;
		info[5]=itno;
		info[6]=stop;
		info[7]=nfev;
		info[8]=njev;
		info[9]=nlss;
	}

	//sba_print_sol(n, m, p, cnp, pnp, x, mnp, &idxij, rcidxs, rcsubs);
	retval=(stop!=7)?  itno : SBA_ERROR;

freemem_and_return: /* NOTE: this point is also reached via a goto! */

	/* free whatever was allocated */
	free(W);   free(U);  free(V);
	free(e);   free(eab);
	free(E);   free(Yj); free(YWt);
	free(S);   free(dp); free(Wtda);
	free(rcidxs); free(rcsubs);
#ifndef SBA_DESTROY_COVS
	if(wght) free(wght);
#else
	/* nothing to do */
#endif /* SBA_DESTROY_COVS */

	free(hx); free(diagUV); free(pdp);
	if(fdj_data.hxx){ // cleanup
		free(fdj_data.hxx);
		free(fdj_data.func_rcidxs);
	}

	sba_crsm_free(&idxij);

	/* free the memory allocated by the matrix inversion & linear solver routines */
	if(matinv) (*matinv)(NULL, 0);
	if(linsolver) (*linsolver)(NULL, NULL, NULL, 0, 0);

	return retval;
}

} // namespace slam
} // namespace hub

#endif // defined HAVE_OPENCV2 && CV_MINOR_VERSION >= 2
