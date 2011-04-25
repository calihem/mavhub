// Statistics: mean, variance

#ifndef _STAT_MEANVAR_H_
#define _STAT_MEANVAR_H_

#include <vector>

namespace mavhub {
	class Stat_MeanVar {
	public:
		Stat_MeanVar(int order_);
		virtual ~Stat_MeanVar();

		/// update statistics with current value
		void update(double x_);
		/// update heuristics
		void update_heur(double x_);
		/// get state: mean
		double get_mean();
		/// get state: var
		double get_var();
		/// get state: var enhanced
		double get_var_e();
	private:
		/// order
		int order;
		/// index
		int index;
		/// data
		std::vector<double> x;
		/// mean
		double mean;
		/// plain variance
		double var;
		/// heuristically enhanced variance
		double var_e;
	};
}

#endif
