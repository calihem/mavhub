// parameter class

#ifndef _CTRL_MHPARAM_H_
#define _CTRL_MHPARAM_H_

namespace mavhub {
	class Mhparam {
	public:
		Mhparam(double value_);
		virtual ~Mhparam();
		void pset(double value_);
		double pget();
	private:
		double value;
	};
}

#endif
