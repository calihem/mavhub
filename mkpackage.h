#ifndef _MKPACKAGE_H_
#define _MKPACKAGE_H_

#include <inttypes.h> //uint8_t
#include <vector>

namespace mavhub {
	/// MKPackage
	class MKPackage {
		public:
			MKPackage(uint8_t addr, uint8_t cmd, const uint8_t *data = 0, int length = 0);
			MKPackage(uint8_t addr, uint8_t cmd, int numofdata, ...);
			MKPackage(const uint8_t *data, int length) throw(const char*);
			virtual ~MKPackage();
			
			uint8_t *rawData();
			int rawSize();
			bool checkCRC() const;

		private:
			uint8_t addr;
			uint8_t cmd;
			std::vector<uint8_t> payload;
			std::vector<uint8_t> encoded;
			
			void encode();
			void decode() throw(const char*);
	};

	// ----------------------------------------------------------------------------
	// MKPackage
	// ----------------------------------------------------------------------------
	inline uint8_t* MKPackage::rawData() { return &encoded[0]; }
	inline int MKPackage::rawSize() { return encoded.size(); }
} // namespace mavhub

#endif
